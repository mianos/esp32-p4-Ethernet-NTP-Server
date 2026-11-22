/* The interface between a time source and the NTP responder. */
#pragma once

#include <cstdint>
#include <format>
#include <ctime>
#include <string>

/* What a reply may claim. */
struct TimeReference {
    bool     locked   = false; /* false => stratum 16 + leap indicator 3 */
    uint8_t  stratum  = 16;    /* 1 with a real reference; 3 from upstream SNTP */
    uint32_t refId    = 0;     /* NTP reference id, host order: store32() writes it */
    time_t   lastSync = 0;     /* when the reference last agreed with us */

    /* Leap indicator 0-2 while locked (1: 61-second minute, 2: 59); the
     * responder sends 3 when not locked. */
    uint8_t leap = 0;

    /* The reference's own error budget in ns. dispersionNs == 0 means neither
     * is stated, and the responder uses its upstream defaults (1 ms, 15 ppm). */
    uint32_t rootDelayNs  = 0;
    uint32_t dispersionNs = 0;

    /* Locked apart from the error budget, which is too large; locked is then
     * false. For logs and /healthz. */
    bool demoted = false;
};

/* "INIT", the refid for an unsynchronised server (RFC 5905 7.3). */
constexpr uint32_t kRefIdInit = (static_cast<uint32_t>('I') << 24) | ('N' << 16) | ('I' << 8) | 'T';

/* Text at stratum 0, 1 and 16, an IPv4 address otherwise (RFC 5905 7.3). */
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

/* Implemented by the upstream SNTP clock (ntp_server.cpp) and the GPS
 * (gps_status.cpp). */
class TimeRef {
public:
    virtual ~TimeRef() = default;

    virtual TimeReference reference() const = 0;

    /* The instant to stamp a reply with. */
    virtual void now(struct timespec &ts) const = 0;

    /* One phrase for the boot log: "upstream 192.0.2.1", "GPS/PPS". */
    virtual std::string describe() const = 0;
};
