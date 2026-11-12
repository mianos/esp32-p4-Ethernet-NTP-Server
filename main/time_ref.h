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
 */
#pragma once

#include <cstdint>
#include <ctime>
#include <string>

/* What the responder may claim, and what it needs to say it. */
struct TimeReference {
    bool     locked   = false; /* false => stratum 16 + leap indicator 3 */
    uint8_t  stratum  = 16;    /* 1 with a real reference; 3 from upstream SNTP */
    uint32_t refId    = 0;     /* NTP reference id, host order: store32() writes it */
    time_t   lastSync = 0;     /* when the reference last agreed with us */
};

/* "INIT" in ASCII. RFC 5905 7.3: a server that cannot vouch for its clock says
 * so in the refid field, and a client that reads it knows the stratum 16 it was
 * sent with was not a mistake. */
constexpr uint32_t kRefIdInit =
    (static_cast<uint32_t>('I') << 24) | ('N' << 16) | ('I' << 8) | 'T';

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
