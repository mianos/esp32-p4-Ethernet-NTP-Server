/*
 * What the responder may claim, and where the time comes from.
 *
 * These three declarations used to live in ntp_server.h, which was fine while
 * there was one implementation. There are two now -- the upstream SNTP clock
 * and the GPS -- and a time source should not have to include the responder in
 * order to be one, so the interface gets its own header and the responder
 * includes it.
 *
 * The interface is three calls wide on purpose. A stratum, a timestamp, and one
 * phrase for the boot log: everything needed to answer a client honestly is in
 * the first, including `locked`, which is what turns a stratum-1 claim into
 * "do not use me" the moment the reference goes away.
 *
 * TimeReference carries three more things than stratum and refid, and each one
 * is a claim the packet makes that the reference is the only party able to fill
 * in: the leap indicator (the receiver knows, the responder cannot guess), and
 * the reference's own delay and dispersion (a GPS with a PPS line measures
 * itself in microseconds; an upstream SNTP server that is only ever polled
 * cannot be measured at all, so it says nothing and the responder falls back to
 * its LAN guesses). A field the reference leaves at zero means exactly that --
 * not stated -- never "zero error".
 */
#pragma once

#include <cstdint>
#include <format>
#include <ctime>
#include <string>

/* What the responder may claim, and what it needs to say it. */
struct TimeReference {
    bool     locked   = false; /* false => stratum 16 + leap indicator 3 */
    uint8_t  stratum  = 16;    /* 1 with a real reference; 3 from upstream SNTP */
    uint32_t refId    = 0;     /* NTP reference id, host order: store32() writes it */
    time_t   lastSync = 0;     /* when the reference last agreed with us */

    /* Leap indicator, 0-2, and meaningful only while locked: the responder
     * substitutes 3 ("clock not synchronised") itself when it is not. 1 means
     * the last minute of the UTC day has 61 seconds, 2 means 59. Zero is both
     * "no warning" and "no opinion", which are the same thing to a client. */
    uint8_t leap = 0;

    /* The reference's own error budget, in nanoseconds, with 0 for "not
     * stated". The responder then falls back to its LAN guesses -- 1 ms of
     * round trip, 15 ppm of ageing -- which is the honest thing to say about an
     * upstream nobody measured. A reference that can measure itself states
     * microseconds instead, so that a client watching root dispersion sees a
     * holdover getting worse rather than a number that never moves. The two are
     * read together: dispersionNs == 0 means rootDelayNs is not stated either. */
    uint32_t rootDelayNs  = 0;
    uint32_t dispersionNs = 0;

    /* Set when the reference is holding together but its own budget says it is
     * no longer fit to serve: locked is then already false and stratum 16, and
     * this is only the reason, for the log and for /healthz. A reference that
     * knows it is out and keeps advertising a lock is worse than one that
     * admits it, because the client has no way to check. */
    bool demoted = false;
};

/* "INIT" in ASCII. RFC 5905 7.3: a server that cannot vouch for its clock says
 * so in the refid field, and a client that reads it knows the stratum 16 it was
 * sent with was not a mistake. */
constexpr uint32_t kRefIdInit = (static_cast<uint32_t>('I') << 24) | ('N' << 16) | ('I' << 8) | 'T';

/* The refid field is four bytes that mean different things depending on the
 * stratum: text ("GPS", "INIT", "RATE") at stratum 1 and for the kiss codes,
 * an IPv4 address above stratum 1 (RFC 5905 section 7.3). Print whichever it
 * is -- a hexadecimal blob answers none of the questions asked of it. */
inline std::string refIdText(uint32_t refId, uint8_t stratum)
{
    char bytes[4];
    for (int i = 0; i < 4; ++i) {
        bytes[i] = static_cast<char>((refId >> (24 - 8 * i)) & 0xFF);
    }

    if (stratum <= 1 || stratum >= 16) {
        std::string text(bytes, sizeof(bytes));
        while (!text.empty() && text.back() == '\0') {
            text.pop_back();
        }
        return text;
    }

    return std::format("{}.{}.{}.{}", static_cast<uint8_t>(bytes[0]),
                       static_cast<uint8_t>(bytes[1]), static_cast<uint8_t>(bytes[2]),
                       static_cast<uint8_t>(bytes[3]));
}

/* Where the time comes from. Two implementations: the upstream SNTP clock
 * (ntp_server.cpp) and the GPS with its PPS line (gps_time.cpp). */
class TimeRef {
public:
    virtual ~TimeRef() = default;

    virtual TimeReference reference() const = 0;

    /* The instant to stamp a reply with. */
    virtual void now(struct timespec &ts) const = 0;

    /* One phrase for the boot log: "upstream 192.0.2.1", "GPS/PPS". */
    virtual std::string describe() const = 0;
};
