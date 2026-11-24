/* The leap path: what UBX-NAV-TIMELS said, whether that announcement is in
 * force, and the indicator a reply carries.
 *
 * Three decisions the standards fix and a receiver can get wrong -- the fields
 * and their two independent valid bits, RFC 5905's last-minute window, and what
 * an unlocked or kiss-o'-death reply says instead -- kept as functions of plain
 * values with no ESP-IDF headers and no state, so they can be exercised on a host
 * (test/leap_test.cpp). The callers own the locking and the storage; nothing here
 * reads a member. */
#pragma once

#include <cstddef>
#include <cstdint>

namespace leap {

/* UBX-NAV-TIMELS version 0, 24 bytes: currLs (i1) at 9, lsChange (i1) at 11,
 * timeToLsEvent (i4, LSB first) at 12, and `valid` at 23, where bit 0 says the
 * GPS-UTC offset is known and bit 1 says the event time is. The two are separate
 * because a receiver that has just started up can have one and not the other. */
constexpr std::size_t kTimelsLen = 24;

/* What the message said, not what it means: a field the message did not vouch
 * for is reported as it arrived, and the gate below is what ignores it. */
struct Announcement {
    bool    offsetKnown = false;
    int8_t  offsetS     = 0; /* GPS-UTC, e.g. 18 */
    bool    eventKnown  = false;
    int8_t  changeS     = 0; /* +1 a 61-second minute, -1 a 59-second one */
    int32_t inS         = 0; /* seconds to the event; negative once it is past */
};

/* False, with `out` untouched, when the payload is shorter than the version this
 * reads. The frame decoder already drops a NAV-TIMELS that is not 24 bytes, so
 * this is the handler not depending on that. */
inline bool parseTimels(const uint8_t *payload, std::size_t len, Announcement &out)
{
    if (payload == nullptr || len < kTimelsLen) {
        return false;
    }
    Announcement a;
    a.offsetKnown = (payload[23] & 0x01) != 0;
    a.offsetS     = static_cast<int8_t>(payload[9]);
    a.eventKnown  = (payload[23] & 0x02) != 0;
    a.changeS     = static_cast<int8_t>(payload[11]);
    a.inS         = static_cast<int32_t>(static_cast<uint32_t>(payload[12]) |
                                 (static_cast<uint32_t>(payload[13]) << 8) |
                                 (static_cast<uint32_t>(payload[14]) << 16) |
                                 (static_cast<uint32_t>(payload[15]) << 24));
    out = a;
    return true;
}

/* NTP's leap indicator: 1 when the last minute of the day runs to 61 seconds, 2
 * when it ends at 59, 0 when nothing is announced. RFC 5905 puts the warning on
 * the final minute, which is 0 < timeToLsEvent <= 60; past the event the receiver
 * counts negative and there is nothing left to warn about. Only one second in
 * either direction exists, so a change field carrying anything else is not an
 * announcement -- answering +2 with indicator 2 would put a 59-second minute on
 * the wire that no standard defines. */
inline uint8_t indicator(const Announcement &a)
{
    if (!a.offsetKnown || !a.eventKnown || a.inS <= 0 || a.inS > 60) {
        return 0;
    }
    if (a.changeS == 1) {
        return 1;
    }
    return a.changeS == -1 ? 2 : 0;
}

/* NTP leap indicator values: the warning above, and "clock not synchronised". */
constexpr uint8_t kNoWarning = 0;
constexpr uint8_t kAlarm     = 3;

/* The indicator a reply carries: the announcement while locked, 3 when the source
 * is not locked, and 0 on a kiss-o'-death so a client cannot read the rate
 * limit's answer as a leap warning. */
inline uint8_t forReply(bool locked, uint8_t announced, bool kod)
{
    if (kod) {
        return kNoWarning;
    }
    return locked ? announced : kAlarm;
}

/* TAI-UTC, which PTP announces in its flags and its stamps: TAI = GPS + 19 s and
 * NAV-TIMELS reports GPS-UTC, so the sum is what PTP wants. 18 s is assumed until
 * the receiver reports an offset, and 127 is where an octet stops. */
inline uint8_t taiMinusUtc(int gpsMinusUtc)
{
    const int offset = (gpsMinusUtc > 0 ? gpsMinusUtc : 18) + 19;
    return static_cast<uint8_t>(offset > 127 ? 127 : offset);
}

}  // namespace leap
