#include "ntp_server.h"

#include "ptp.h"

#include "clock_source.h"
#include "ntp_control.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "lwip/inet.h"
#include "lwip/ip6_addr.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

#include "gps_time.h"
#include "ntp_time.h"
#include "serve_path.h"
#include "tx_lag.h"
#include "settings.h"
#include "text.h"

namespace {

/* ---- Shape of the conversation ---------------------------------------- */

constexpr int          kPort             = 123;
constexpr size_t       kPacketBytes      = 48;   /* NTPv4 header, no MAC */
/* Raw-frame replies, with the hardware transmit stamp. */
/* Raw-frame replies with the hardware transmit stamp. Off is the old
 * software-only path, kept because it is the fallback when a stamp is missing. */
constexpr bool         kRawSend          = true;
constexpr uint32_t     kReceiveTimeoutMs = 1000; /* how often the task looks up */
constexpr uint32_t     kReportEveryMs    = 60000;
/* 12288, not 4096: every request copies a GpsTime::Status by value (the
 * reference's error budget and lock state come out of it) and status() parses
 * the antenna delay with from_chars<double>, whose fast_float path alone is
 * several hundred bytes. At 4096 the stack-protector tripped inside
 * from_chars within a minute of the first client, and the board rolled back.
 * The control path then added a 483-octet request buffer and a reply buffer of
 * up to 1295 (a long variable list in one piece), which is where the rest of
 * the headroom went. */
constexpr int          kTaskStack        = 12288;
/* Above the status logger and just under lwIP (18): a woken responder should
 * preempt now rather than at the next scheduling point. It runs for
 * microseconds per packet, so nothing is starved by it. */
constexpr UBaseType_t  kTaskPriority     = 17;

constexpr size_t       kStampBytes    = 8;
constexpr size_t       kOriginOffset  = 24;
constexpr size_t       kReceiveOffset = 32;
constexpr size_t       kTransmitOffset = 40;

/* ---- Protocol constants ----------------------------------------------- */

/* Seconds between 1900-01-01 and 1970-01-01. */
constexpr uint32_t kNtpUnixDelta = 2208988800u;

constexpr uint8_t kModeClient   = 3; /* the only data mode this device answers */
constexpr uint8_t kModeServer   = 4;
constexpr uint8_t kModeControl  = 6; /* ntpq: read-only, see ntp_control.cpp */

constexpr uint8_t kStratumUnlocked = 16; /* "unsynchronised": ignore this server */
constexpr uint8_t kLeapNoWarning   = 0;
constexpr uint8_t kLeapAlarm      = 3; /* "clock not synchronised" */

/* 2^-20 s, roughly one microsecond: the granularity this clock can actually
 * distinguish, as opposed to a made-up number of digits. */
constexpr uint8_t kPrecision = 0xEC;

/* Stratum while the upstream is the reference. The upstream's own stratum is
 * not visible through lwIP's SNTP client, so this claims one hop more than a
 * well-behaved internet server rather than one fewer than reality: a client
 * that under-estimates the distance to us is the one that gets hurt. */
constexpr uint8_t kStratumFromUpstream = 3;

constexpr uint32_t kRefIdRate = (static_cast<uint32_t>('R') << 24) | ('A' << 16) | ('T' << 8) | 'E';

/* Root delay and dispersion, 16.16 fixed point. The delay is a standing guess:
 * the round trip to the upstream is not something lwIP exposes, and 1 ms is a
 * fair LAN number. The dispersion starts from the same 1 ms and grows with age
 * (see rootDispersion()), so it never claims a better path than it knows. */
constexpr uint32_t kRootDelay = 65; /* 1 ms */

/* Cheap-crystal ageing, 15 ppm: after an hour without the reference this is
 * 54 ms of claimed uncertainty, which is what a client needs in order to
 * demote this server instead of trusting a clock nobody is correcting. */
constexpr double kDispersionPpm = 15.0;

/* ---- Rate limiting ----------------------------------------------------
 * A LAN full of clients polling at minpoll is fine; one client in a loop is
 * not. Each source address gets a small bucket: the burst is answered, then
 * one token per few seconds, then a single KoD RATE per second, then silence.
 * Only eight sources are remembered, which is a deliberate bound -- the oldest
 * entry is recycled, so a flood from many addresses degrades to answering the
 * newest ones rather than growing the table.
 *
 * The burst is 16 because that is what a *person* testing with sntp costs: sntp
 * fires five queries per run, so two runs back to back are ten packets, and a
 * limit that bites during ordinary manual testing teaches the wrong lesson. The
 * sustained rate is what actually protects the device, and 1 per 2 s is already
 * ~30x more permissive than any real daemon (chronyd, timesyncd and ntpd all
 * poll every 32-1024 s).
 */

constexpr int      kBucketDepth    = 16;
constexpr uint32_t kRefillEveryMs  = 2000;
constexpr int      kBucketSlots    = 8;
constexpr uint32_t kKodIntervalMs  = 1000;
constexpr size_t   kAddressKeySize = 16;

enum class Action { Answer, Kod, Drop };

/* ---- Wire format helpers ---------------------------------------------- */

void store32(uint8_t *out, uint32_t value)
{
    out[0] = static_cast<uint8_t>(value >> 24);
    out[1] = static_cast<uint8_t>(value >> 16);
    out[2] = static_cast<uint8_t>(value >> 8);
    out[3] = static_cast<uint8_t>(value);
}

uint32_t load32(const uint8_t *in)
{
    return (static_cast<uint32_t>(in[0]) << 24) | (static_cast<uint32_t>(in[1]) << 16) |
           (static_cast<uint32_t>(in[2]) << 8) | static_cast<uint32_t>(in[3]);
}

/* A stamp back to nanoseconds since the epoch, or 0 when the field is empty (a
 * client that sent no origin has nothing to compare against). */
int64_t stampToNs(const uint8_t *stamp)
{
    const uint32_t seconds = load32(stamp);
    if (seconds == 0) {
        return 0;
    }
    const uint64_t fraction = load32(stamp + 4);
    return (static_cast<int64_t>(seconds) - static_cast<int64_t>(kNtpUnixDelta)) * 1000000000 +
           static_cast<int64_t>((fraction * 1000000000ull) >> 32);
}

/* 32-bit seconds since 1900 followed by a 32-bit binary fraction. The seconds
 * field wraps in 2036 and the era is implied -- that is the format, not an
 * oversight: a client and server that disagree about the era are already
 * 68 years apart and no 48-byte reply can fix it. */
void writeTimestamp(uint8_t *out, const struct timespec &ts)
{
    const uint32_t seconds =
        static_cast<uint32_t>(static_cast<int64_t>(ts.tv_sec) + static_cast<int64_t>(kNtpUnixDelta));
    const uint32_t fraction =
        static_cast<uint32_t>((static_cast<uint64_t>(ts.tv_nsec) << 32) / 1000000000ull);
    store32(out, seconds);
    store32(out + 4, fraction);
}

void writeTimestamp(uint8_t *out, time_t when)
{
    struct timespec ts = {};
    ts.tv_sec = when;
    writeTimestamp(out, ts);
}

/* 16.16 fixed point, which resolves about 15 us. A microsecond-accurate
 * reference therefore rounds *up* to one least significant bit rather than
 * claiming zero uncertainty -- the field cannot say 10 us, and 15 is closer to
 * the truth than 0. */
uint32_t toFixed(double seconds)
{
    const double fixed = std::ceil(seconds * 65536.0);
    if (fixed <= 0.0) {
        return 0;
    }
    return fixed >= 65535.0 ? 0xFFFFu : static_cast<uint32_t>(fixed);
}

/* Root delay and dispersion, before the 16.16 field rounds them: what the
 * reference states about itself if it can, the generic LAN guesses if it
 * cannot. The difference is the point -- the GPS measures microseconds and says
 * so, the upstream SNTP server has no way to measure the round trip and says
 * 1 ms rather than pretend, and a holdover grows the GPS's number until a
 * client can see it degrade. */
struct RootUncertainty {
    double delayS      = 0.0;
    double dispersionS = 0.0;
};

RootUncertainty rootUncertainty(const TimeReference &ref, time_t now)
{
    if (ref.dispersionNs != 0) {
        return {static_cast<double>(ref.rootDelayNs) * 1e-9,
                static_cast<double>(ref.dispersionNs) * 1e-9};
    }

    /* Ageing at the rate of a cheap crystal: after an hour without the
     * reference this is 54 ms of claimed uncertainty, which is what a client
     * needs in order to demote this server instead of trusting a clock nobody
     * is correcting. */
    const double lanS = static_cast<double>(kRootDelay) / 65536.0;
    RootUncertainty figures{lanS, lanS};
    if (ref.lastSync != 0 && now > ref.lastSync) {
        figures.dispersionS += static_cast<double>(now - ref.lastSync) * kDispersionPpm * 1e-6;
    }
    return figures;
}

/* Rate limiting is per address, so a v4 client arriving over the dual-stack
 * socket (AF_INET6, v4-mapped) and one arriving over a v4-only build are keyed
 * differently -- harmless, and cheaper than normalising. */
void addressKey(const sockaddr_storage &src, uint8_t out[kAddressKeySize])
{
    std::memset(out, 0, kAddressKeySize);
    if (src.ss_family == AF_INET) {
        const auto *v4 = reinterpret_cast<const sockaddr_in *>(&src);
        std::memcpy(out, &v4->sin_addr, 4);
#if CONFIG_LWIP_IPV6
    } else if (src.ss_family == AF_INET6) {
        const auto *v6 = reinterpret_cast<const sockaddr_in6 *>(&src);
        std::memcpy(out, &v6->sin6_addr, 16);
#endif
    }
}

/* "192.0.2.50:123" or "[2001:db8::1]:123", for the log only. */
std::string sourceText(const sockaddr_storage &src)
{
    if (src.ss_family == AF_INET) {
        const auto    *v4 = reinterpret_cast<const sockaddr_in *>(&src);
        const uint8_t *b  = reinterpret_cast<const uint8_t *>(&v4->sin_addr);
        return std::format("{}.{}.{}.{}:{}", b[0], b[1], b[2], b[3], ntohs(v4->sin_port));
    }
#if CONFIG_LWIP_IPV6
    if (src.ss_family == AF_INET6) {
        const auto *v6 = reinterpret_cast<const sockaddr_in6 *>(&src);
        ip6_addr_t  lwip{};
        std::memcpy(lwip.addr, &v6->sin6_addr, sizeof(lwip.addr));
        /* ip6addr_ntoa_r is lwIP's own formatter and wants an array, so this one
         * keeps its buffer; everything around it does not need one. */
        char addr[IP6ADDR_STRLEN_MAX] = {};
        ip6addr_ntoa_r(&lwip, addr, sizeof(addr));
        return std::format("[{}]:{}", addr, ntohs(v6->sin6_port));
    }
#endif
    return "?";
}

/* ---- Interleaved mode (RFC 9769) --------------------------------------
 *
 * A client that can is asking for the transmit timestamp of the *previous*
 * reply rather than a prediction of this one, which is the only way a server
 * with a software transmit stamp can hand out a timestamp it did not have to
 * guess. The negotiation is implicit and rides in the timestamp fields: an
 * interleaved request carries, as its origin timestamp, the receive timestamp
 * of the reply before it, and the reply then carries the saved transmit
 * timestamp of that same exchange.
 *
 * The RFC requires a fixed-length queue of receive/transmit pairs, keyed by IP
 * and not by port (clients move ports), with old entries dropped, and receive
 * timestamps that are unique across the queue. The queue is small on purpose: it
 * only has to outlive one round trip, and a client that falls further behind
 * than this simply gets a basic reply, which every client understands.
 *
 * Timestamps are kept as the eight wire octets. Nothing here needs to interpret
 * them, and echoing the exact bytes is what makes the client's match work.
 */
constexpr size_t   kPairSlots      = 24;
constexpr uint32_t kPairMaxAgeMs   = 120000; /* a client polling slower than this starts again */
constexpr size_t   kPairBytes      = 8;

struct SavedPair {
    uint8_t  key[kAddressKeySize] = {};
    uint8_t  wire[kPairBytes]     = {}; /* the T3 that went out, which is the cookie */
    uint8_t  request[kPairBytes]  = {}; /* that request's T1, returned as the origin */
    uint8_t  receive[kPairBytes]  = {}; /* that exchange's T2, returned as the receive */
    uint8_t  transmit[kPairBytes] = {}; /* that exchange's T3, returned as the transmit */
    uint32_t atMs                 = 0;
    bool     used                 = false;
};

/* One fraction unit, about a quarter of a nanosecond: enough to make a
 * timestamp differ without moving it by anything measurable. */
void bumpFraction(uint8_t stamp[kPairBytes])
{
    for (int i = static_cast<int>(kPairBytes) - 1; i >= 4; --i) {
        if (++stamp[i] != 0) {
            return;
        }
    }
}

/* The queue is keyed by the transmit timestamp that actually went on the wire,
 * which is what the client echoes as its origin. A reply carries the previous
 * exchange's stamps; the cookie names which exchange that was.
 *
 * Note that the wire transmit and the fresh one differ in exactly the mode this
 * exists for: the reply hands back the saved T3, so the cookie the client will
 * send is the saved one, not the one just taken. */
class Interleaved {
public:
    /* Does `origin` name a saved exchange with this address? The RFC forbids
     * answering in the mode unless it does, and forbids answering twice to the
     * same exchange, so a match is consumed here. */
    bool take(const sockaddr_storage &from, const uint8_t origin[kPairBytes],
              uint8_t transmitOut[kPairBytes], uint8_t originOut[kPairBytes],
              uint8_t receiveOut[kPairBytes], uint32_t nowMs)
    {
        if (isZero(origin, kPairBytes)) {
            return false;
        }
        uint8_t key[kAddressKeySize];
        addressKey(from, key);

        for (SavedPair &slot : m_slots) {
            if (!slot.used || nowMs - slot.atMs > kPairMaxAgeMs) {
                continue;
            }
            if (std::memcmp(slot.key, key, kAddressKeySize) != 0 ||
                std::memcmp(slot.wire, origin, kPairBytes) != 0) {
                continue;
            }
            std::memcpy(transmitOut, slot.transmit, kPairBytes);
            std::memcpy(originOut, slot.request, kPairBytes);
            std::memcpy(receiveOut, slot.receive, kPairBytes);
            slot.used = false;
            ++m_taken;
            return true;
        }
        return false;
    }

    /* Every answered request saves its exchange, interleaved or not: a client
     * has to be able to become interleaved, and that needs one saved exchange. */
    void save(const sockaddr_storage &from, const uint8_t wire[kPairBytes],
              const uint8_t request[kPairBytes], const uint8_t transmit[kPairBytes],
              const uint8_t receive[kPairBytes], uint32_t nowMs)
    {
        SavedPair *victim = nullptr;
        for (SavedPair &slot : m_slots) {
            if (!slot.used) {
                victim = &slot;
                break;
            }
        }
        if (victim == nullptr) {
            victim = &m_slots[m_next];
            m_next = (m_next + 1) % kPairSlots;
        }
        ++m_saved;
        addressKey(from, victim->key);
        std::memcpy(victim->wire, wire, kPairBytes);
        std::memcpy(victim->request, request, kPairBytes);
        std::memcpy(victim->transmit, transmit, kPairBytes);
        std::memcpy(victim->receive, receive, kPairBytes);
        victim->atMs = nowMs;
        victim->used = true;
    }

private:
    static bool isZero(const uint8_t *bytes, size_t len)
    {
        for (size_t i = 0; i < len; ++i) {
            if (bytes[i] != 0) {
                return false;
            }
        }
        return true;
    }

    SavedPair m_slots[kPairSlots];
public:
    uint32_t m_taken = 0;
    uint32_t m_saved = 0;
    size_t   m_next  = 0;
};

/* ---- Rate limiting ---- */
class RateLimiter {
public:
    Action check(const sockaddr_storage &src, uint32_t nowMs)
    {
        uint8_t key[kAddressKeySize];
        addressKey(src, key);

        Bucket *slot = find(key);
        if (slot == nullptr) {
            slot = recycle();
            std::memcpy(slot->key, key, sizeof(key));
            slot->tokens       = kBucketDepth;
            slot->lastRefillMs = nowMs;
            slot->lastKodMs    = 0;
        }
        slot->lastSeenMs = nowMs;

        /* Refill one token per interval, never more than the burst depth: a
         * client that has been quiet does not bank credit for a flood. */
        const uint32_t elapsed = nowMs - slot->lastRefillMs;
        if (elapsed >= kRefillEveryMs) {
            const uint32_t gained = elapsed / kRefillEveryMs;
            slot->tokens = std::min<int>(kBucketDepth, slot->tokens + static_cast<int>(gained));
            slot->lastRefillMs += gained * kRefillEveryMs;
        }

        if (slot->tokens > 0) {
            --slot->tokens;
            return Action::Answer;
        }
        /* Over the limit: say why once a second, then say nothing. A responder
         * that keeps answering KoDs as fast as it is asked has not limited
         * anything. */
        if (slot->lastKodMs == 0 || (nowMs - slot->lastKodMs) >= kKodIntervalMs) {
            slot->lastKodMs = nowMs;
            return Action::Kod;
        }
        return Action::Drop;
    }

private:
    struct Bucket {
        uint8_t  key[kAddressKeySize] = {};
        int      tokens               = 0;
        bool     used                 = false;
        uint32_t lastRefillMs         = 0;
        uint32_t lastKodMs            = 0;
        uint32_t lastSeenMs           = 0;
    };

    Bucket *find(const uint8_t key[kAddressKeySize])
    {
        for (Bucket &slot : m_slots) {
            if (slot.used && std::memcmp(slot.key, key, kAddressKeySize) == 0) {
                return &slot;
            }
        }
        return nullptr;
    }

    /* Oldest first: on a LAN the working set is small, and dropping the source
     * that has been quiet longest is the least wrong thing to do. */
    Bucket *recycle()
    {
        Bucket *oldest = &m_slots[0];
        for (Bucket &slot : m_slots) {
            if (!slot.used) {
                slot.used = true;
                return &slot;
            }
            if (slot.lastSeenMs < oldest->lastSeenMs) {
                oldest = &slot;
            }
        }
        return oldest;
    }

    Bucket m_slots[kBucketSlots];
};

struct Request {
    uint8_t version               = 0;
    uint8_t mode                  = 0;
    uint8_t poll                  = 0;
    uint8_t origin[kStampBytes]   = {}; /* the client's cookie, echoed or matched */
    uint8_t receive[kStampBytes]  = {}; /* the client's own previous arrival time */
    uint8_t transmit[kStampBytes] = {};
};

/* Worth answering as a data request? Long enough to hold a header, a version
 * both ends speak, and in client mode. Replying to a mode 4 packet is how you
 * build an infinite conversation between two servers, so mode 4 is silence --
 * as is mode 7, the remote facility, which has nothing here to configure. Mode
 * 6 never reaches this: it is read-only control traffic and ntp_control.cpp
 * answers it. Extension fields past byte 48 are ignored and never echoed. */
bool parseRequest(const uint8_t *packet, int len, Request &req)
{
    if (len < static_cast<int>(kPacketBytes)) {
        return false;
    }

    req.version = static_cast<uint8_t>((packet[0] >> 3) & 0x07);
    req.mode    = static_cast<uint8_t>(packet[0] & 0x07);
    if (req.version != 3 && req.version != 4) {
        return false;
    }
    if (req.mode != kModeClient) {
        return false;
    }

    /* Echoed verbatim, as RFC 5905 requires: a client uses its own poll back
     * to decide what it asked for, and v4 clients check it on a KoD. */
    req.poll = packet[2];
    std::memcpy(req.origin, packet + kOriginOffset, kStampBytes);
    std::memcpy(req.receive, packet + kReceiveOffset, kStampBytes);
    std::memcpy(req.transmit, packet + kTransmitOffset, kStampBytes);
    return true;
}

/* The reply is always exactly 48 bytes, whatever arrived: an NTP server that
 * sends more than it receives is a reflector, and 48 is all a client needs to
 * compute offset and delay. */
/* `originStamp` and `transmitStamp` are eight wire octets each, or null for the
 * two default behaviours: echo the request's transmit timestamp, and stamp the
 * transmit field with `transmitted`. Interleaved replies supply both. */
void buildReply(uint8_t *out, const Request &req, const TimeReference &ref, bool kod,
                const struct timespec &received, const struct timespec &transmitted,
                const uint8_t *originStamp = nullptr, const uint8_t *transmitStamp = nullptr,
                const struct timespec *transmitLive = nullptr,
                const uint8_t *receiveStamp = nullptr)
{
    const uint8_t  stratum = kod ? 0 : ref.stratum;
    /* The reference's own leap indicator while it is locked, "not
     * synchronised" when it is not, and no warning attached to a KoD: a client
     * told to slow down has not been told anything about leap seconds. */
    const uint8_t  leap    = kod ? kLeapNoWarning : (ref.locked ? ref.leap : kLeapAlarm);
    const uint32_t refId   = kod ? kRefIdRate : ref.refId;

    const RootUncertainty uncertainty = rootUncertainty(ref, received.tv_sec);
    const uint32_t        delay       = toFixed(uncertainty.delayS);
    const uint32_t        dispersion  = toFixed(uncertainty.dispersionS);

    /* The version echoes the request's: that is what ntpd does and what RFC 5905
     * asks for, and a v3-only client is entitled to reject a v4 reply. */
    out[0] = static_cast<uint8_t>((leap << 6) | ((req.version & 0x07) << 3) | kModeServer);
    out[1] = stratum;
    out[2] = req.poll;
    out[3] = kPrecision;
    store32(out + 4, delay);
    store32(out + 8, dispersion);
    store32(out + 12, refId);

    /* Never synchronised means a zero reference timestamp (RFC 5905 7.3), not
     * the Unix epoch dressed up in NTP seconds: a client that sees stratum 16
     * also expects to see nothing claimed here. */
    if (ref.lastSync != 0) {
        writeTimestamp(out + 16, ref.lastSync);
    } else {
        store32(out + 16, 0);
        store32(out + 20, 0);
    }
    /* The origin timestamp exists to let the client match the reply to its own
     * request, so it goes back byte for byte -- even if the client sent zero,
     * which some do on the first packet. An interleaved reply echoes the
     * request's *receive* field instead, which is what tells the client the
     * reply is in that mode. */
    if (originStamp != nullptr) {
        std::memcpy(out + kOriginOffset, originStamp, kStampBytes);
    } else {
        std::memcpy(out + kOriginOffset, req.transmit, kStampBytes);
    }
    if (receiveStamp != nullptr) {
        std::memcpy(out + kReceiveOffset, receiveStamp, kStampBytes);
    } else {
        writeTimestamp(out + kReceiveOffset, received);
    }
    if (transmitStamp != nullptr) {
        std::memcpy(out + kTransmitOffset, transmitStamp, kStampBytes);
    } else if (transmitLive != nullptr) {
        writeTimestamp(out + kTransmitOffset, *transmitLive);
    } else {
        writeTimestamp(out + kTransmitOffset, transmitted);
    }
}

/* The upstream SNTP clock, dressed as a TimeRef. */
class UpstreamTimeRef : public TimeRef {
public:
    TimeReference reference() const override
    {
        NtpTime &ntp = NtpTime::instance();

        TimeReference ref;
        ref.lastSync = ntp.lastSync();
        /* "Synced" only records ever having been set, so the freshness window
         * is what makes this a lock: an upstream that went away an hour ago
         * leaves a clock that is drifting with no one correcting it. The
         * window lives in ClockSource because the arbiter needs the same
         * answer, and two definitions of "still good" would be one too many.
         * The client's own reach register is deliberately not consulted -- it
         * flaps on a single lost poll, and it is empty for a clock set by
         * POST /ntp/sync while timeSource is off. */
        ref.locked = ClockSource::instance().upstreamLocked();
        if (!ref.locked) {
            ref.stratum = kStratumUnlocked;
            ref.refId   = kRefIdInit;
            return ref;
        }
        ref.stratum = kStratumFromUpstream;
        ref.refId   = upstreamRefId();
        return ref;
    }

    void now(struct timespec &ts) const override
    {
        /* The clock NtpTime stepped. Whatever disciplines the clock later --
         * GPS/PPS, or the EMAC's 1588 timestamp unit -- the responder still
         * reads the system clock; discipline is somebody else's job, which is
         * exactly what this interface is for. */
        clock_gettime(CLOCK_REALTIME, &ts);
    }

    std::string describe() const override
    {
        const std::optional<std::string> name = NtpTime::instance().upstreamName();
        return name ? "upstream " + *name : std::string("no upstream");
    }

private:
    static bool parseDottedQuad(const std::string &text, uint32_t &out)
    {
        unsigned a = 0, b = 0, c = 0, d = 0;
        /* Exactly four fields: from_chars rejects the trailing character that
         * sscanf's %c was there to catch, and rejects "1.2.3.4.5" too. */
        if (!text::getList(text, '.', a, b, c, d)) {
            return false;
        }
        if (a > 255 || b > 255 || c > 255 || d > 255) {
            return false;
        }
        out = (a << 24) | (b << 16) | (c << 8) | d;
        return true;
    }

    /* RFC 5905 puts the upstream's IPv4 address in refid above stratum 1, or
     * the first four bytes of its MD5 hash when it was named rather than
     * addressed. lwIP keeps the resolved address to itself, so a hostname gets
     * a stable FNV-1a of the name instead: above stratum 1 this field is
     * informational (it is how ntpd spots a loop), and FNV-1a keeps it stable
     * across reboots without dragging a hash implementation into the image. */
    static uint32_t upstreamRefId()
    {
        const std::optional<std::string> name = NtpTime::instance().upstreamName();
        if (!name) {
            return kRefIdInit;
        }

        uint32_t quad = 0;
        if (parseDottedQuad(*name, quad)) {
            return quad;
        }

        uint32_t hash = 2166136261u;
        for (const char c : *name) {
            hash ^= static_cast<uint8_t>(c);
            hash *= 16777619u;
        }
        return hash;
    }
};

}  // namespace

/* Is the upstream the one serving? One predicate, because the reply and the
 * control view must not be able to disagree about who the reference is.
 *
 * Under "auto" the receiver is served for as long as it still claims something
 * worth serving -- locked, or holding over on its fitted rate, which is what its
 * own reference reports -- and the upstream takes the reply only once the client
 * has actually synced. While neither holds, the receiver answers stratum 16,
 * which is honest and is the same reply it gives with no upstream at all. */
bool servingUpstream()
{
    switch (Settings::instance().timeSource()) {
    case Settings::TimeSource::Gps:
        return false;
    case Settings::TimeSource::Auto:
        return ClockSource::instance().owner() == ClockOwner::Upstream;
    case Settings::TimeSource::Dhcp:
    case Settings::TimeSource::Static:
    case Settings::TimeSource::Off:
        return true;
    }
    return true;
}

/* Which reference is served is a setting, and a setting can change while the
 * responder is running (POST /config); swapping the TimeRef the serving task is
 * using would hand it a dangling pointer. So the object it holds never changes:
 * it asks Settings on every call and forwards to one of two stateless wrappers,
 * which is also what keeps `serveNtp` and the clock source independent. */
class SelectedTimeRef : public TimeRef {
public:
    TimeReference reference() const override { return active().reference(); }

    void now(struct timespec &ts) const override { active().now(ts); }

    std::string describe() const override { return active().describe(); }

private:
    static TimeRef &active()
    {
        static const std::unique_ptr<TimeRef> upstream = makeUpstreamTimeRef();
        static const std::unique_ptr<TimeRef> gps      = makeGpsTimeRef();
        return servingUpstream() ? *upstream : *gps;
    }
};

std::unique_ptr<TimeRef> makeUpstreamTimeRef()
{
    return std::unique_ptr<TimeRef>(new UpstreamTimeRef());
}

std::unique_ptr<TimeRef> makeTimeRef()
{
    return std::unique_ptr<TimeRef>(new SelectedTimeRef());
}

NtpServer &NtpServer::instance()
{
    static NtpServer server;
    return server;
}

void NtpServer::taskEntry(void *arg)
{
    static_cast<NtpServer *>(arg)->serveLoop();
}

esp_err_t NtpServer::begin()
{
    if (m_running.load(std::memory_order_relaxed)) {
        return ESP_OK;
    }
    if (m_task != nullptr) {
        /* Still shutting down after serveNtp went off: starting a second task
         * here is how two of them end up on one socket. The next lease retries. */
        ESP_LOGW(TAG, "responder task is still stopping");
        return ESP_ERR_INVALID_STATE;
    }
    if (m_sock >= 0) {
        /* Left over from a stop that timed out: nothing owns it now. */
        close(m_sock);
        m_sock = -1;
    }
    if (!Settings::instance().serveNtp()) {
        ESP_LOGI(TAG, "not serving: serveNtp is off");
        return ESP_OK;
    }

#if CONFIG_LWIP_IPV6
    /* One socket for both families: an AF_INET6 socket bound to :: with
     * v6only off takes IPv4 clients too, as v4-mapped addresses. Same trick
     * the HTTP server uses, and it keeps /healthz's two-address promise. */
    const int family = AF_INET6;
#else
    const int family = AF_INET;
#endif

    const int sock = socket(family, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket: errno %d", errno);
        return ESP_FAIL;
    }

    /* The task must wake up regularly enough to notice serveNtp going off and
     * to print its report; a blocking recv would do neither. */
    struct timeval timeout = {};
    timeout.tv_sec         = kReceiveTimeoutMs / 1000;
    timeout.tv_usec        = static_cast<long>(kReceiveTimeoutMs % 1000) * 1000;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) {
        ESP_LOGW(TAG, "SO_RCVTIMEO: errno %d", errno);
    }

#if CONFIG_LWIP_IPV6
    const int v6only = 0;
    if (setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &v6only, sizeof(v6only)) != 0) {
        /* Not fatal: if dual-stack is compiled out, this socket answers IPv6
         * only, and the log is the only way to find out. */
        ESP_LOGW(TAG, "IPV6_V6ONLY=0 failed (errno %d): no IPv4 clients", errno);
    }
    sockaddr_in6 addr = {};
    addr.sin6_family  = AF_INET6;
    addr.sin6_port    = htons(kPort);
    addr.sin6_addr    = in6addr_any;
#else
    sockaddr_in addr  = {};
    addr.sin_family   = AF_INET;
    addr.sin_port     = htons(kPort);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
#endif

    if (bind(sock, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind udp/%d: errno %d", kPort, errno);
        close(sock);
        return ESP_FAIL;
    }

    if (!m_ref) {
        m_ref = makeTimeRef();
    }

    m_sock = sock;
    m_running.store(true, std::memory_order_relaxed);
    if (xTaskCreate(&NtpServer::taskEntry, "ntp_serve", kTaskStack, this, kTaskPriority, &m_task) !=
        pdPASS) {
        m_running.store(false, std::memory_order_relaxed);
        m_sock = -1;
        close(sock);
        m_task = nullptr;
        ESP_LOGE(TAG, "cannot start the responder task");
        return ESP_ERR_NO_MEM;
    }

    /* Measured, not assumed: see tx_lag.cpp. Failing to start it only costs the
     * correction, so it is logged and not fatal. */
    if (TxLag::instance().begin() != ESP_OK) {
        ESP_LOGW(TAG, "no transmit-lag probe: reply stamps stay uncorrected");
    }

    const TimeReference ref = m_ref->reference();
    ESP_LOGI(TAG, "serving on udp/%d (%s), reference: %s, stratum %u%s", kPort,
#if CONFIG_LWIP_IPV6
             "dual-stack",
#else
             "IPv4",
#endif
             m_ref->describe().c_str(), ref.stratum,
             ref.locked ? "" : " (not locked: advertising stratum 16)");
    return ESP_OK;
}

esp_err_t NtpServer::applySettings()
{
    if (Settings::instance().serveNtp()) {
        return begin();
    }

    if (!m_running.exchange(false, std::memory_order_relaxed)) {
        return ESP_OK;
    }
    ESP_LOGW(TAG, "serveNtp is off: stopping the responder");
    /* The task is asleep in recvfrom() for at most a second; wait for it so a
     * quick off/on toggle cannot end up with two of them. The socket is this
     * side's to close -- the task only ever reads it -- so closing it here
     * cannot pull the descriptor out from under a task that is still running. */
    const int sock = m_sock;
    for (int i = 0; i < 30 && m_task != nullptr; ++i) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (m_task != nullptr) {
        /* Still inside recvfrom(): leave the socket alone and let the next
         * begin() clean it up rather than closing a descriptor in use. */
        ESP_LOGW(TAG, "responder task has not stopped yet");
        return ESP_ERR_TIMEOUT;
    }
    m_sock = -1;
    if (sock >= 0) {
        close(sock);
    }
    return ESP_OK;
}

TimeReference NtpServer::reference() const
{
    return m_ref ? m_ref->reference() : TimeReference{};
}

NtpServer::Counters NtpServer::counters() const
{
    Counters c;
    c.answered  = m_answered.load(std::memory_order_relaxed);
    c.kod       = m_kod.load(std::memory_order_relaxed);
    c.dropped   = m_dropped.load(std::memory_order_relaxed);
    c.perMinute = static_cast<double>(m_ratePerMin.load(std::memory_order_relaxed));
    const int64_t lastServed = m_lastServedUs.load(std::memory_order_relaxed);
    if (lastServed >= 0) {
        c.lastServedS = static_cast<double>(esp_timer_get_time() - lastServed) / 1e6;
    }
    c.clients = m_clients.load(std::memory_order_relaxed);
    return c;
}

NtpServer::RootFigures NtpServer::wireRootFigures() const
{
    const RootUncertainty uncertainty = rootUncertainty(reference(), std::time(nullptr));
    return {uncertainty.delayS * 1e6, uncertainty.dispersionS * 1e6};
}

void NtpServer::fillControlFacts(ControlFacts &facts, const TimeReference &ref) const
{
    facts.upstream = servingUpstream();
    if (facts.upstream) {
        if (const std::optional<std::string> name = NtpTime::instance().upstreamName()) {
            facts.peer = *name;
        }
    } else {
        /* The address a reference clock has in ntpd. ntpq prints it as the
         * remote, and sys.peeradr names it. */
        facts.peer = "127.127.20.0";
    }
    facts.leap     = ref.leap;
    facts.locked   = ref.locked;
    facts.stratum  = ref.stratum;
    facts.refId    = ref.refId;
    facts.lastSync = ref.lastSync;

    /* The same two figures the reply carries, so a client reading either gets
     * one answer rather than two. */
    const RootUncertainty uncertainty = rootUncertainty(ref, std::time(nullptr));
    facts.rootDelayNs  = static_cast<int64_t>(uncertainty.delayS * 1e9);
    facts.dispersionNs = static_cast<int64_t>(uncertainty.dispersionS * 1e9);

    const GpsTime::Status gps = GpsTime::instance().status();
    facts.jitterNs = gps.jitterNs;
    facts.offsetNs = gps.offsetNs;

    facts.answered        = m_answered.load(std::memory_order_relaxed);
    facts.kod             = m_kod.load(std::memory_order_relaxed);
    facts.dropped         = m_dropped.load(std::memory_order_relaxed);
    facts.requests        = facts.answered + facts.kod + facts.dropped;
    facts.controlRequests = m_control.load(std::memory_order_relaxed);
    facts.uptimeS         = esp_timer_get_time() / 1000000;
    if (const esp_app_desc_t *desc = esp_app_get_description()) {
        facts.version = desc->version;
    }
}

void NtpServer::recordServePath(const sockaddr_storage &from, const struct timespec &received,
                                const uint8_t *origin)
{
    const int64_t nowUs = esp_timer_get_time();

    uint32_t       addr = 0;
    bool           ok   = true;
    const uint8_t *b    = nullptr;
    if (from.ss_family == AF_INET) {
        addr = reinterpret_cast<const sockaddr_in *>(&from)->sin_addr.s_addr;
    } else if (from.ss_family == AF_INET6) {
        b                = reinterpret_cast<const sockaddr_in6 *>(&from)->sin6_addr.s6_addr;
        const bool mapped = b[10] == 0xFF && b[11] == 0xFF;
        for (int i = 0; i < 10 && mapped; ++i) {
            if (b[i] != 0) {
                ok = false;
            }
        }
        if (mapped && ok) {
            std::memcpy(&addr, b + 12, sizeof(addr));
        } else {
            ok = false;
        }
    } else {
        ok = false;
    }
    if (!ok) {
        ++m_serveWhy[1];
        reportServePath(nowUs);
        return;
    }

    const int64_t clientNs = stampToNs(origin);
    if (clientNs == 0) {
        ++m_serveWhy[2];
        reportServePath(nowUs);
        return;
    }
    const int64_t delta =
        (static_cast<int64_t>(received.tv_sec) * 1000000000 + received.tv_nsec) - clientNs;
    if (delta < -1000000000 || delta > 1000000000) {
        ++m_serveWhy[3];
        reportServePath(nowUs);
        return;
    }
    ++m_serveWhy[0];

    ServePathSlot *slot   = nullptr;
    ServePathSlot *oldest = &m_serve[0];
    for (auto &s : m_serve) {
        if (s.addr == addr) {
            slot = &s;
            break;
        }
        if (s.lastUs < oldest->lastUs) {
            oldest = &s;
        }
    }
    if (slot == nullptr) {
        slot        = oldest;
        slot->addr  = addr;
        slot->count = 0;
    }
    if (slot->count == 0) {
        slot->minNs = delta;
        slot->maxNs = delta;
    } else {
        slot->minNs = std::min(slot->minNs, delta);
        slot->maxNs = std::max(slot->maxNs, delta);
    }
    ++slot->count;
    slot->lastUs = nowUs;

    reportServePath(nowUs);
}

void NtpServer::reportServePath(int64_t nowUs)
{
    if (nowUs - m_serveReportUs < 60000000) {
        return;
    }
    m_serveReportUs = nowUs;
    ESP_LOGI(TAG, "servepath kept %u family %u origin-zero %u out-of-range %u",
             static_cast<unsigned>(m_serveWhy[0]), static_cast<unsigned>(m_serveWhy[1]),
             static_cast<unsigned>(m_serveWhy[2]), static_cast<unsigned>(m_serveWhy[3]));
    m_serveWhy[0] = m_serveWhy[1] = m_serveWhy[2] = m_serveWhy[3] = 0;
    /* One claim for the minute, from the median client rather than the widest.
     * The range a client reports says as much about that client as about us: a
     * slow poller, a busy machine or a stepping clock can double it, and with
     * the maximum they all set the claim for everyone. The median keeps the
     * honest majority. */
    int64_t  halves[kServeSlots] = {};
    int      clients             = 0;
    uint32_t samples             = 0;
    for (auto &s : m_serve) {
        if (s.count == 0) {
            continue;
        }
        const uint8_t *ip    = reinterpret_cast<const uint8_t *>(&s.addr);
        const int64_t  halfNs = (s.maxNs - s.minNs) / 2;
        ESP_LOGI(TAG, "servepath %u.%u.%u.%u n %u half %.1f us centre %.1f us", ip[0], ip[1],
                 ip[2], ip[3], static_cast<unsigned>(s.count),
                 static_cast<double>(halfNs) / 1000.0,
                 static_cast<double>(s.minNs + s.maxNs) / 2000.0);
        halves[clients++] = halfNs;
        samples += s.count;
        s.count = 0;
    }
    if (clients == 0) {
        return;
    }
    for (int i = 1; i < clients; ++i) {
        const int64_t v = halves[i];
        int           j = i;
        while (j > 0 && halves[j - 1] > v) {
            halves[j] = halves[j - 1];
            --j;
        }
        halves[j] = v;
    }
    /* The lower middle when the count is even: with two clients the upper one
     * is simply the worse of the pair, which is the maximum this was written to
     * avoid. */
    ServePath::instance().observe(halves[(clients - 1) / 2], samples);
}

void NtpServer::serveLoop()
{
    /* This task's socket, taken by value: the member may be replaced by a
     * restart, and a task that closed "m_sock" on the way out would then close
     * its successor's socket. */
    const int   sock = m_sock;
    RateLimiter limiter;
    Interleaved pairs;
    uint32_t    answered = 0, kodSent = 0, dropped = 0;      /* cumulative */
    uint32_t    winAnswered = 0, winKod = 0, winDropped = 0; /* since report */
    uint32_t    reportAt    = pdTICKS_TO_MS(xTaskGetTickCount()) + kReportEveryMs;
    bool        firstSeen   = false;

    /* Distinct sources, over the last kRecentSources answered requests. The
     * address only: a client's port changes with every query, so including it
     * would count one poller as many. This answers "is anything else using
     * this", which is otherwise invisible. */
    const auto recordSource = [this](const sockaddr_storage &addr) {
        uint32_t    h = 2166136261u; /* FNV-1a */
        const auto  mix = [&h](const uint8_t *p, std::size_t n) {
            for (std::size_t i = 0; i < n; ++i) {
                h ^= p[i];
                h *= 16777619u;
            }
        };
        if (addr.ss_family == AF_INET) {
            const auto *in = reinterpret_cast<const sockaddr_in *>(&addr);
            mix(reinterpret_cast<const uint8_t *>(&in->sin_addr), sizeof(in->sin_addr));
        } else if (addr.ss_family == AF_INET6) {
            const auto *in6 = reinterpret_cast<const sockaddr_in6 *>(&addr);
            mix(in6->sin6_addr.s6_addr, 16);
        } else {
            return;
        }
        m_recent[m_recentAt++ % kRecentSources] = h;

        uint8_t distinct = 0;
        for (uint8_t i = 0; i < kRecentSources; ++i) {
            if (m_recent[i] == 0) {
                continue;
            }
            bool seen = false;
            for (uint8_t j = 0; j < i && !seen; ++j) {
                seen = m_recent[j] == m_recent[i];
            }
            if (!seen) {
                ++distinct;
            }
        }
        m_clients.store(distinct, std::memory_order_relaxed);
    };

    while (m_running.load(std::memory_order_relaxed)) {
        sockaddr_storage from    = {};
        socklen_t        fromLen = sizeof(from);

        /* Room for extension fields, so a long request is still parsed from its
         * first 48 bytes instead of being dropped as unrecognised -- and for a
         * whole control request, which is a header plus up to 468 octets of
         * variable names. Truncating one of those makes its last name partial,
         * and a partial name is an unknown variable. */
        uint8_t   packet[12 + kControlChunk + 3] = {};
        const int len = recvfrom(sock, packet, sizeof(packet), 0,
                                 reinterpret_cast<sockaddr *>(&from), &fromLen);

        /* The arrival stamp the MAC took, if it took one. This is the receive
         * timestamp: the frame was stamped before the driver task was woken, so
         * neither the scheduler nor the stack is in it. Without one -- the PHC
         * off, or an entry aged out -- this falls back to a read after the
         * syscall, which is the old behaviour and carries both. */
        uint8_t  peer[16]  = {};
        size_t   peerLen   = 0;
        uint16_t peerPort  = 0;
        if (from.ss_family == AF_INET) {
            std::memcpy(peer, &reinterpret_cast<const sockaddr_in *>(&from)->sin_addr, 4);
            peerLen  = 4;
            peerPort = ntohs(reinterpret_cast<const sockaddr_in *>(&from)->sin_port);
        } else if (from.ss_family == AF_INET6) {
            /* The socket is dual-stack, so an IPv4 client arrives as a mapped
             * address (::ffff:a.b.c.d) on an AF_INET6 sockaddr. The ring holds
             * the address the frame actually carried, so the mapped form has to
             * be unwrapped or the lookup compares four bytes with sixteen. */
            const auto    *v6 = reinterpret_cast<const sockaddr_in6 *>(&from);
            const uint8_t *a  = reinterpret_cast<const uint8_t *>(&v6->sin6_addr);
            bool           mapped = true;
            for (int i = 0; i < 10; ++i) {
                if (a[i] != 0) {
                    mapped = false;
                    break;
                }
            }
            if (mapped && a[10] == 0xff && a[11] == 0xff) {
                std::memcpy(peer, a + 12, 4);
                peerLen = 4;
            } else {
                std::memcpy(peer, a, 16);
                peerLen = 16;
            }
            peerPort = ntohs(v6->sin6_port);
        }
        ptp::Rx       hwRx;
        const bool    hardware = len >= 0 && peerLen != 0 &&
                                 ptp::takeRx(peer, peerLen, peerPort, &hwRx);

        static bool toldStamp = false;
        if (len >= 0 && !toldStamp) {
            toldStamp = true;
            ESP_LOGI(TAG, "arrival stamp: %s family %d peerLen %u port %u peer0 %02x%02x",
                     hardware ? "hardware" : "software", static_cast<int>(from.ss_family),
                     static_cast<unsigned>(peerLen), static_cast<unsigned>(peerPort), peer[0],
                     peer[1]);
        }

        struct timespec received = {};
        /* The stamp is in the 1588 counter's base; the reply has to be in the
         * reference's. In the GPS case those are the same clock and the
         * conversion is a few ns; with an upstream the counter is a free-running
         * copy set once at boot, and the difference is whatever the upstream
         * has stepped the system clock by. Reading both around the same instant
         * makes it a subtraction either way. */
        int64_t toServed = 0;
        if (hardware) {
            struct timespec servedNow = {};
            m_ref->now(servedNow);
            toServed = static_cast<int64_t>(servedNow.tv_sec) * 1000000000LL +
                       servedNow.tv_nsec - ptp::nowNs();
            const int64_t t2 = hwRx.ns + toServed;
            received.tv_sec  = static_cast<time_t>(t2 / 1000000000);
            received.tv_nsec = static_cast<long>(t2 % 1000000000);
        } else {
            m_ref->now(received);
        }

        const uint32_t nowMs = pdTICKS_TO_MS(xTaskGetTickCount());

        if (len < 0) {
            /* The 1 s timeout is the normal way round this loop. */
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGW(TAG, "recvfrom: errno %d", errno);
            }
        } else if ((packet[0] & 0x07) == kModeControl) {
            /* Control messages share the socket but not the protocol, and are
             * answered ahead of the client path and outside the rate limiter:
             * one ntpq -p run is three packets, and a KoD is not a shape it
             * expects for a control query. Read-only, so metering them buys
             * little. */
            const TimeReference ref = m_ref->reference();
            ControlFacts        facts;
            fillControlFacts(facts, ref);
            m_control.fetch_add(1, std::memory_order_relaxed);

            /* A long variable list is split; the client reassembles by offset
             * and gives up on holes, so the fragments go out back to back. */
            uint8_t  control[kControlReplyBytes] = {};
            uint16_t total                       = 0;
            uint16_t offset                      = 0;
            for (int fragment = 0; fragment < 8; ++fragment) {
                const size_t controlLen =
                    buildControlReply(control, sizeof(control), packet, static_cast<size_t>(len),
                                      facts, offset, total);
                if (controlLen == 0 ||
                    sendto(sock, control, controlLen, 0,
                           reinterpret_cast<const sockaddr *>(&from), fromLen) < 0) {
                    if (controlLen != 0) {
                        ESP_LOGW(TAG, "sendto (control): errno %d", errno);
                    }
                    break;
                }
                offset = static_cast<uint16_t>(
                    offset + std::min<size_t>(kControlChunk, total - offset));
                if (offset >= total) {
                    break;
                }
            }
        } else {
            Request req;
            if (!parseRequest(packet, len, req)) {
                ++dropped;
                ++winDropped;
                m_dropped.store(dropped, std::memory_order_relaxed);
            } else {
                const Action action = limiter.check(from, nowMs);
                if (action == Action::Drop) {
                    ++dropped;
                    ++winDropped;
                    m_dropped.store(dropped, std::memory_order_relaxed);
                } else {
                    const bool kod = action == Action::Kod;

                    /* How the served time is moving, seen from a client. Its
                     * own send stamp is in the transmit field of the request --
                     * the origin field is empty on a plain client and only
                     * carries an echo from an interleaved one. The difference
                     * between that stamp and ours is the client's clock offset
                     * plus the path, so over a minute the constant cancels and
                     * the range left is drift, measured on answers that went
                     * out rather than guessed by a probe. */
                    if (!kod) {
                        recordServePath(from, received, req.transmit);
                    }

                    /* Take both stamps around the reply build, which is as
                     * close to the wire as a software timestamp gets. */
                    const TimeReference ref         = m_ref->reference();
                    struct timespec    transmitted = {};
                    m_ref->now(transmitted);
                    /* Whatever the transmit path costs, the stamp is early by
                     * it. Correcting for it is the mirror of the receive lag
                     * the dispersion already admits, and it is what keeps a
                     * client's offset from carrying the difference.
                     *
                     * Not on the raw path: the estimator is calibrated against
                     * a socket sendto(), and a frame handed straight to the DMA
                     * descriptor is on the wire in a fraction of that. Applying
                     * the socket figure there over-corrects by its own size,
                     * which is the whole 300 us arriving as a 150 us offset. */
                    if (!hardware) {
                        m_txCorrected.fetch_add(1, std::memory_order_relaxed);
                        transmitted.tv_nsec += TxLag::instance().lagNs();
                        while (transmitted.tv_nsec >= 1000000000L) {
                            transmitted.tv_nsec -= 1000000000L;
                            ++transmitted.tv_sec;
                        }
                    }

                    uint8_t receiveStamp[kStampBytes] = {};
                    uint8_t liveStamp[kStampBytes]    = {};
                    writeTimestamp(receiveStamp, received);
                    writeTimestamp(liveStamp, transmitted);

                    /* Interleaved mode, if this client is asking for it and we
                     * still hold the exchange it names. RFC 9769 forbids
                     * answering in the mode unless both hold, and forbids
                     * reusing a receive timestamp, so the lookup consumes it. */
                    uint8_t savedTransmit[kStampBytes] = {};
                    uint8_t savedOrigin[kStampBytes]   = {};
                    uint8_t savedReceive[kStampBytes]  = {};
                    bool interleavedMode = false;
                    if (!kod && std::memcmp(req.receive, req.transmit, kStampBytes) != 0) {
                        m_ilTry.fetch_add(1, std::memory_order_relaxed);
                        interleavedMode = pairs.take(from, req.origin, savedTransmit, savedOrigin,
                                                     savedReceive, nowMs);
                        if (interleavedMode) {
                            m_ilHit.fetch_add(1, std::memory_order_relaxed);
                        }
                    }

                    uint8_t reply[kPacketBytes] = {};
                    if (interleavedMode) {
                        m_interleaved.fetch_add(1, std::memory_order_relaxed);
                        buildReply(reply, req, ref, kod, received, transmitted, savedOrigin,
                                   savedTransmit, nullptr, savedReceive);
                    } else {
                        buildReply(reply, req, ref, kod, received, transmitted, nullptr, nullptr,
                                   &transmitted);
                    }

                    /* Off the wire with a hardware stamp when one is available.
                     * The stamp this frame gets is what an interleaved client's
                     * *next* request is answered with, so it is measured rather
                     * than corrected -- which is the whole point of the saved
                     * pair. The frame's own transmit field still has to be
                     * written before the send, so it stays the software stamp
                     * plus the estimator; a 1588 one-step unit would fix that
                     * and this one cannot. */
                    int64_t       measuredTx = 0;
                    uint8_t       measuredStamp[kStampBytes] = {};
                    const int64_t replySent                  = esp_timer_get_time();
                    int           replyRc                    = -1;
                    if (kRawSend && hardware && ptp::sendUdp(peer, peerLen, peerPort, hwRx.mac, reply,
                                                 sizeof(reply), &measuredTx)) {
                        replyRc = static_cast<int>(sizeof(reply));
                    } else {
                        replyRc = sendto(sock, reply, sizeof(reply), 0,
                                         reinterpret_cast<const sockaddr *>(&from), fromLen);
                    }
                    const int64_t replyCost = (esp_timer_get_time() - replySent) * 1000;
                    if (replyRc < 0) {
                        ESP_LOGW(TAG, "sendto: errno %d", errno);
                    } else if (kod) {
                        ++kodSent;
                        ++winKod;
                        m_kod.store(kodSent, std::memory_order_relaxed);
                    } else {
                        /* Saved for the client's *next* request. The receive
                         * timestamp handed out must never equal the transmit one
                         * in the same reply, or the client cannot tell the modes
                         * apart (RFC 9769 section 2). */
                        if (std::memcmp(receiveStamp, liveStamp, kStampBytes) == 0) {
                            bumpFraction(liveStamp);
                            std::memcpy(reply + kTransmitOffset, liveStamp, kStampBytes);
                        }
                        /* The key is what the client will echo: the transmit
                         * that actually went out, which in this mode is the
                         * saved one rather than the fresh one. */
                        if (measuredTx != 0) {
                            const int64_t servedTx = measuredTx + toServed;
                            struct timespec measured = {};
                            measured.tv_sec  = static_cast<time_t>(servedTx / 1000000000);
                            measured.tv_nsec = static_cast<long>(servedTx % 1000000000);
                            writeTimestamp(measuredStamp, measured);
                        }
                        pairs.save(from, interleavedMode ? savedTransmit : liveStamp,
                                   req.transmit, measuredTx != 0 ? measuredStamp : liveStamp,
                                   receiveStamp, nowMs);
                        TxLag::instance().recordReply(replyCost);
                        ++answered;
                        ++winAnswered;
                        m_answered.store(answered, std::memory_order_relaxed);
                        m_lastServedUs.store(esp_timer_get_time(), std::memory_order_relaxed);
                        recordSource(from);
                        if (!firstSeen) {
                            firstSeen = true;
                            ESP_LOGI(TAG, "first request from %s: stratum %u%s",
                                     sourceText(from).c_str(), ref.stratum,
                                     ref.locked ? "" : " (not locked)");
                        }
                    }
                }
            }

            m_requests.store(answered, std::memory_order_relaxed);
        }

        if (static_cast<int32_t>(nowMs - reportAt) >= 0) {
            /* The rate is published before the window is cleared, so a reader
             * sampling at any point sees the last completed window rather than
             * a count that silently restarts underneath it. */
            m_ratePerMin.store((winAnswered * 60000u) / kReportEveryMs, std::memory_order_relaxed);
            if (winAnswered != 0 || winKod != 0 || winDropped != 0) {
                /* One line a minute at most: the serial console is the only
                 * local debug there is, and a LAN polling at minpoll would
                 * otherwise bury everything else in the log. */
                ESP_LOGI(TAG, "answered %u, KoD %u, dropped %u in the last %u s",
                         winAnswered, winKod, winDropped, kReportEveryMs / 1000);
            }
            m_ilSaved.store(pairs.m_saved, std::memory_order_relaxed);
            m_ilKeyOnly.store(pairs.m_taken, std::memory_order_relaxed);
            winAnswered = winKod = winDropped = 0;
            reportAt = nowMs + kReportEveryMs;
        }
    }

    /* The socket belongs to the owner (applySettings), which closes it once this
     * handle is gone; only the handle is cleared here. */
    m_task = nullptr;
    ESP_LOGI(TAG, "responder stopped");
    vTaskDelete(nullptr);
}
