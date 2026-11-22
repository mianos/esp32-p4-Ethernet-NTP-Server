/* Compiles this file's ESP_LOGD lines in; they stay hidden until raised with
 * the console's `loglevel` command. */
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "ntp/ntp_server.h"

#include "app/cores.h"

#include "ptp/ptp.h"

#include "clock/clock_source.h"
#include "ntp/ntp_control.h"

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

#include "gps/gps_time.h"
#include "clock/ntp_time.h"
#include "ntp/serve_path.h"
#include "ntp/tx_lag.h"
#include "app/settings.h"
#include "app/text.h"

namespace {

/* ---- Shape of the conversation ---------------------------------------- */

constexpr int    kPort        = 123;
constexpr size_t kPacketBytes = 48; /* NTPv4 header, no MAC */
/* Send replies as raw frames on the 1588 path when the request was hardware
 * stamped; otherwise, or when false, through sendto(). */
constexpr bool     kRawSend          = true;
constexpr uint32_t kReceiveTimeoutMs = 1000; /* how often the task looks up */
constexpr uint32_t kReportEveryMs    = 60000;
/* Mode 6 copies a GpsTime::Status by value and uses request and reply buffers
 * of about 0.5 and 1.3 KB; 4096 overflowed. */
constexpr int kTaskStack = 12288;
/* Just under lwIP (18), so a woken responder runs at once. */
constexpr UBaseType_t kTaskPriority = 17;

constexpr size_t kStampBytes     = 8;
constexpr size_t kOriginOffset   = 24;
constexpr size_t kReceiveOffset  = 32;
constexpr size_t kTransmitOffset = 40;

/* ---- Protocol constants ----------------------------------------------- */

/* Seconds between 1900-01-01 and 1970-01-01. */
constexpr uint32_t kNtpUnixDelta = 2208988800u;

constexpr uint8_t kModeClient  = 3; /* the only data mode this device answers */
constexpr uint8_t kModeServer  = 4;
constexpr uint8_t kModeControl = 6; /* ntpq: read-only, see ntp_control.cpp */

constexpr uint8_t kStratumUnlocked = 16; /* "unsynchronised": ignore this server */
constexpr uint8_t kLeapNoWarning   = 0;
constexpr uint8_t kLeapAlarm       = 3; /* "clock not synchronised" */

/* 2^-20 s, about 1 us. */
constexpr uint8_t kPrecision = 0xEC;

/* Stratum while the upstream is the reference. lwIP's SNTP client does not
 * expose the upstream's stratum, so this assumes a stratum 2 upstream. */
constexpr uint8_t kStratumFromUpstream = 3;

constexpr uint32_t kRefIdRate = (static_cast<uint32_t>('R') << 24) | ('A' << 16) | ('T' << 8) | 'E';

/* Upstream root delay, 16.16: lwIP does not expose the round trip, so 1 ms is
 * assumed. Dispersion starts there and grows with age (rootUncertainty()). */
constexpr uint32_t kRootDelay = 65; /* 1 ms */

/* Dispersion growth since the last upstream sync: 15 ppm, 54 ms an hour. */
constexpr double kDispersionPpm = 15.0;

/* ---- Rate limiting ----------------------------------------------------
 * A token bucket per source address: a burst of 16 (a few manual sntp runs),
 * then one every 2 s, then one KoD RATE a second, then silence. Eight sources
 * are tracked; the least recently seen is recycled. */

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

/* An NTP timestamp as Unix ns, or 0 for an empty field. */
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

/* 32-bit seconds since 1900 and a 32-bit fraction; the era is implied. */
void writeTimestamp(uint8_t *out, const struct timespec &ts)
{
    const uint32_t seconds = static_cast<uint32_t>(static_cast<int64_t>(ts.tv_sec) +
                                                   static_cast<int64_t>(kNtpUnixDelta));
    const uint32_t fraction =
        static_cast<uint32_t>((static_cast<uint64_t>(ts.tv_nsec) << 32) / 1000000000ull);
    store32(out, seconds);
    store32(out + 4, fraction);
}

void writeTimestamp(uint8_t *out, time_t when)
{
    struct timespec ts = {};
    ts.tv_sec          = when;
    writeTimestamp(out, ts);
}

/* 16.16 fixed point (about 15 us per bit), rounded up so a small figure is
 * never sent as zero. */
uint32_t toFixed(double seconds)
{
    const double fixed = std::ceil(seconds * 65536.0);
    if (fixed <= 0.0) {
        return 0;
    }
    return fixed >= 65535.0 ? 0xFFFFu : static_cast<uint32_t>(fixed);
}

/* The reference's own figures when it states them (GPS), otherwise the
 * upstream defaults above. */
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

    const double    lanS = static_cast<double>(kRootDelay) / 65536.0;
    RootUncertainty figures{lanS, lanS};
    if (ref.lastSync != 0 && now > ref.lastSync) {
        figures.dispersionS += static_cast<double>(now - ref.lastSync) * kDispersionPpm * 1e-6;
    }
    return figures;
}

/* The source address as a 16-byte key; v4-mapped addresses are not
 * normalised. */
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
        char addr[IP6ADDR_STRLEN_MAX] = {};
        ip6addr_ntoa_r(&lwip, addr, sizeof(addr));
        return std::format("[{}]:{}", addr, ntohs(v6->sin6_port));
    }
#endif
    return "?";
}

/* ---- Interleaved mode (RFC 9769) --------------------------------------
 *
 * An interleaved request carries, as its origin, the receive timestamp of our
 * previous reply; the answer carries that reply's measured transmit timestamp.
 * Pairs are kept per IP address (not port), as wire octets, for one round trip;
 * a client that misses gets a basic reply. */
constexpr size_t   kPairSlots    = 24;
constexpr uint32_t kPairMaxAgeMs = 120000; /* a client polling slower than this starts again */
constexpr size_t   kPairBytes    = 8;

struct SavedPair {
    uint8_t  key[kAddressKeySize] = {};
    uint8_t  receive[kPairBytes]  = {}; /* the T2 that went out, which is the cookie */
    uint8_t  transmit[kPairBytes] = {}; /* the same reply's T3, as measured */
    uint32_t atMs                 = 0;
    bool     used                 = false;
};

bool isZeroStamp(const uint8_t stamp[kPairBytes])
{
    return std::all_of(stamp, stamp + kPairBytes, [](uint8_t b) { return b == 0; });
}

/* Add one fraction unit (~0.23 ns). */
void bumpFraction(uint8_t stamp[kPairBytes])
{
    for (int i = static_cast<int>(kPairBytes) - 1; i >= 4; --i) {
        if (++stamp[i] != 0) {
            return;
        }
    }
}

/* Keyed by the receive timestamp, which an interleaved client echoes; a basic
 * client echoes the transmit timestamp. */
class Interleaved {
public:
    /* True if `origin` names a reply sent to this address; a match is used
     * once, as the RFC requires. */
    bool take(const sockaddr_storage &from, const uint8_t origin[kPairBytes],
              uint8_t transmitOut[kPairBytes], uint32_t nowMs)
    {
        uint8_t key[kAddressKeySize];
        addressKey(from, key);

        for (SavedPair &slot : m_slots) {
            if (!slot.used || nowMs - slot.atMs > kPairMaxAgeMs) {
                continue;
            }
            if (std::memcmp(slot.key, key, kAddressKeySize) != 0 ||
                std::memcmp(slot.receive, origin, kPairBytes) != 0) {
                continue;
            }
            std::memcpy(transmitOut, slot.transmit, kPairBytes);
            slot.used = false;
            ++m_taken;
            return true;
        }
        return false;
    }

    /* Every answered request saves its pair, so a client can switch modes. */
    void save(const sockaddr_storage &from, const uint8_t receive[kPairBytes],
              const uint8_t transmit[kPairBytes], uint32_t nowMs)
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
        std::memcpy(victim->receive, receive, kPairBytes);
        std::memcpy(victim->transmit, transmit, kPairBytes);
        victim->atMs = nowMs;
        victim->used = true;
    }

    SavedPair m_slots[kPairSlots];
    uint32_t  m_taken = 0;
    uint32_t  m_saved = 0;
    size_t    m_next  = 0;
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
        /* Over the limit: one KoD a second, otherwise silence. */
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

    /* An unused slot, else the least recently seen. */
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

/* A client-mode (3) request of version 3 or 4 and at least 48 bytes. Other
 * modes are not answered (mode 6 is handled before this). Extension fields are
 * ignored. */
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

    /* Echoed, as RFC 5905 requires. */
    req.poll = packet[2];
    std::memcpy(req.origin, packet + kOriginOffset, kStampBytes);
    std::memcpy(req.receive, packet + kReceiveOffset, kStampBytes);
    std::memcpy(req.transmit, packet + kTransmitOffset, kStampBytes);
    return true;
}

/* Always 48 bytes, never more than the request. A non-null `savedTransmit`
 * (the previous reply's measured transmit, as wire octets) makes it an
 * interleaved reply. */
void buildReply(uint8_t *out, const Request &req, const TimeReference &ref, bool kod,
                const struct timespec &received, const struct timespec &transmitted,
                const uint8_t *savedTransmit = nullptr)
{
    const uint8_t stratum = kod ? 0 : ref.stratum;
    /* Leap 3 when not locked; 0 on a KoD. */
    const uint8_t  leap  = kod ? kLeapNoWarning : (ref.locked ? ref.leap : kLeapAlarm);
    const uint32_t refId = kod ? kRefIdRate : ref.refId;

    const RootUncertainty uncertainty = rootUncertainty(ref, received.tv_sec);
    const uint32_t        delay       = toFixed(uncertainty.delayS);
    const uint32_t        dispersion  = toFixed(uncertainty.dispersionS);

    /* The request's version, as RFC 5905 asks. */
    out[0] = static_cast<uint8_t>((leap << 6) | ((req.version & 0x07) << 3) | kModeServer);
    out[1] = stratum;
    out[2] = req.poll;
    out[3] = kPrecision;
    store32(out + 4, delay);
    store32(out + 8, dispersion);
    store32(out + 12, refId);

    /* Zero reference timestamp when never synchronised (RFC 5905 7.3). */
    if (ref.lastSync != 0) {
        writeTimestamp(out + 16, ref.lastSync);
    } else {
        store32(out + 16, 0);
        store32(out + 20, 0);
    }
    /* Origin: the request's transmit, byte for byte (even zero); an
     * interleaved reply echoes its receive field instead. */
    std::memcpy(out + kOriginOffset, savedTransmit != nullptr ? req.receive : req.transmit,
                kStampBytes);
    writeTimestamp(out + kReceiveOffset, received);
    if (savedTransmit != nullptr) {
        std::memcpy(out + kTransmitOffset, savedTransmit, kStampBytes);
    } else {
        writeTimestamp(out + kTransmitOffset, transmitted);
    }
}

/* The upstream SNTP clock as a TimeRef. */
class UpstreamTimeRef : public TimeRef {
public:
    TimeReference reference() const override
    {
        NtpTime &ntp = NtpTime::instance();

        TimeReference ref;
        ref.lastSync = ntp.lastSync();
        /* Locked means a recent sync (ClockSource holds the window, shared with
         * the arbiter). The SNTP reach register is not used: it flaps on one
         * lost poll, and is empty after POST /ntp/sync. */
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
        /* The system clock, which SNTP sets. */
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
        /* Exactly four fields. */
        if (!text::getList(text, '.', a, b, c, d)) {
            return false;
        }
        if (a > 255 || b > 255 || c > 255 || d > 255) {
            return false;
        }
        out = (a << 24) | (b << 16) | (c << 8) | d;
        return true;
    }

    /* Above stratum 1 the refid is the upstream's IPv4 address. lwIP does not
     * expose the resolved address of a hostname, so a stable FNV-1a of the name
     * is used instead; the field is only used for loop detection. */
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

/* Whether the upstream is the reference served, for replies and mode 6 alike.
 * Under "auto" it is only once ClockSource has handed over to it. */
bool servingUpstream()
{
    switch (Settings::instance().timeSource()) {
    case Settings::TimeSource::Gps:    return false;
    case Settings::TimeSource::Auto:   return ClockSource::instance().owner() == ClockOwner::Upstream;
    case Settings::TimeSource::Dhcp:
    case Settings::TimeSource::Static:
    case Settings::TimeSource::Off:    return true;
    }
    return true;
}

/* Forwards each call to the TimeRef currently selected, so a settings change
 * never swaps the object the responder holds. */
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
        /* Still stopping; the next lease retries. */
        ESP_LOGW(TAG, "responder task is still stopping");
        return ESP_ERR_INVALID_STATE;
    }
    if (m_sock >= 0) {
        /* Left by a stop that timed out. */
        close(m_sock);
        m_sock = -1;
    }
    if (!Settings::instance().serveNtp()) {
        ESP_LOGI(TAG, "not serving: serveNtp is off");
        return ESP_OK;
    }

#if CONFIG_LWIP_IPV6
    /* One AF_INET6 socket with v6only off takes IPv4 as v4-mapped addresses. */
    const int family = AF_INET6;
#else
    const int family = AF_INET;
#endif

    const int sock = socket(family, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket: errno %d", errno);
        return ESP_FAIL;
    }

    /* Wake regularly to notice serveNtp going off and to report. */
    struct timeval timeout = {};
    timeout.tv_sec         = kReceiveTimeoutMs / 1000;
    timeout.tv_usec        = static_cast<long>(kReceiveTimeoutMs % 1000) * 1000;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) {
        ESP_LOGW(TAG, "SO_RCVTIMEO: errno %d", errno);
    }

#if CONFIG_LWIP_IPV6
    const int v6only = 0;
    if (setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &v6only, sizeof(v6only)) != 0) {
        /* Not fatal, but IPv4 clients will not be answered. */
        ESP_LOGW(TAG, "IPV6_V6ONLY=0 failed (errno %d): no IPv4 clients", errno);
    }
    sockaddr_in6 addr = {};
    addr.sin6_family  = AF_INET6;
    addr.sin6_port    = htons(kPort);
    addr.sin6_addr    = in6addr_any;
#else
    sockaddr_in addr     = {};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(kPort);
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
    if (xTaskCreatePinnedToCore(&NtpServer::taskEntry, "ntp_serve", kTaskStack, this, kTaskPriority,
                                &m_task, cores::kWork) != pdPASS) {
        m_running.store(false, std::memory_order_relaxed);
        m_sock = -1;
        close(sock);
        m_task = nullptr;
        ESP_LOGE(TAG, "cannot start the responder task");
        return ESP_ERR_NO_MEM;
    }

    /* Not fatal: only the sendto() path's correction is lost. */
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
    /* Wait for the task (it wakes every second) before closing the socket. */
    const int sock = m_sock;
    for (int i = 0; i < 30 && m_task != nullptr; ++i) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (m_task != nullptr) {
        /* Leave the socket for the next begin() to close. */
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
    c.answered               = m_answered.load(std::memory_order_relaxed);
    c.kod                    = m_kod.load(std::memory_order_relaxed);
    c.dropped                = m_dropped.load(std::memory_order_relaxed);
    c.perMinute              = static_cast<double>(m_ratePerMin.load(std::memory_order_relaxed));
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
        /* ntpd's address for a GPS reference clock. */
        facts.peer = "127.127.20.0";
    }
    facts.leap     = ref.leap;
    facts.locked   = ref.locked;
    facts.stratum  = ref.stratum;
    facts.refId    = ref.refId;
    facts.lastSync = ref.lastSync;

    /* The same figures as the reply. */
    const RootUncertainty uncertainty = rootUncertainty(ref, std::time(nullptr));
    facts.rootDelayNs                 = static_cast<int64_t>(uncertainty.delayS * 1e9);
    facts.dispersionNs                = static_cast<int64_t>(uncertainty.dispersionS * 1e9);

    const GpsTime::Status gps = GpsTime::instance().status();
    facts.jitterNs            = gps.jitterNs;
    facts.offsetNs            = gps.offsetNs;

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
        b                 = reinterpret_cast<const sockaddr_in6 *>(&from)->sin6_addr.s6_addr;
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
    ESP_LOGD(TAG, "servepath kept %u family %u origin-zero %u out-of-range %u",
             static_cast<unsigned>(m_serveWhy[0]), static_cast<unsigned>(m_serveWhy[1]),
             static_cast<unsigned>(m_serveWhy[2]), static_cast<unsigned>(m_serveWhy[3]));
    m_serveWhy[0] = m_serveWhy[1] = m_serveWhy[2] = m_serveWhy[3] = 0;
    /* The median client's half-range, so one noisy client does not set the
     * claim. */
    int64_t  halves[kServeSlots] = {};
    int      clients             = 0;
    uint32_t samples             = 0;
    for (auto &s : m_serve) {
        if (s.count == 0) {
            continue;
        }
        const uint8_t *ip     = reinterpret_cast<const uint8_t *>(&s.addr);
        const int64_t  halfNs = (s.maxNs - s.minNs) / 2;
        ESP_LOGD(TAG, "servepath %u.%u.%u.%u n %u half %.1f us centre %.1f us", ip[0], ip[1], ip[2],
                 ip[3], static_cast<unsigned>(s.count), static_cast<double>(halfNs) / 1000.0,
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
    /* Lower middle for an even count. */
    ServePath::instance().observe(halves[(clients - 1) / 2], samples);
}

void NtpServer::serveLoop()
{
    /* By value: m_sock may belong to a successor by the time this exits. */
    const int   sock = m_sock;
    RateLimiter limiter;
    Interleaved pairs;
    uint32_t    answered = 0, kodSent = 0, dropped = 0;      /* cumulative */
    uint32_t    winAnswered = 0, winKod = 0, winDropped = 0; /* since report */
    uint32_t    reportAt  = pdTICKS_TO_MS(xTaskGetTickCount()) + kReportEveryMs;
    bool        firstSeen = false;

    /* Distinct source addresses (not ports) over the last kRecentSources
     * answers. */
    const auto recordSource = [this](const sockaddr_storage &addr) {
        uint32_t   h   = 2166136261u; /* FNV-1a */
        const auto mix = [&h](const uint8_t *p, std::size_t n) {
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

        /* Big enough for a full mode 6 request (header plus up to 468 octets
         * of names) and for requests with extension fields. */
        uint8_t   packet[12 + kControlChunk + 3] = {};
        const int len                            = recvfrom(sock, packet, sizeof(packet), 0,
                                                            reinterpret_cast<sockaddr *>(&from), &fromLen);

        /* The MAC's arrival stamp if there is one; otherwise the time is read
         * after recvfrom(), which includes the stack and scheduling delay. */
        uint8_t  peer[16] = {};
        size_t   peerLen  = 0;
        uint16_t peerPort = 0;
        if (from.ss_family == AF_INET) {
            std::memcpy(peer, &reinterpret_cast<const sockaddr_in *>(&from)->sin_addr, 4);
            peerLen  = 4;
            peerPort = ntohs(reinterpret_cast<const sockaddr_in *>(&from)->sin_port);
        } else if (from.ss_family == AF_INET6) {
            /* Unwrap ::ffff:a.b.c.d to match the address the frame carried. */
            const auto    *v6     = reinterpret_cast<const sockaddr_in6 *>(&from);
            const uint8_t *a      = reinterpret_cast<const uint8_t *>(&v6->sin6_addr);
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
        ptp::Rx    hwRx;
        const bool hardware =
            len >= 0 && peerLen != 0 && ptp::takeRx(peer, peerLen, peerPort, &hwRx);

        static bool toldStamp = false;
        if (len >= 0 && !toldStamp) {
            toldStamp = true;
            ESP_LOGD(TAG, "arrival stamp: %s family %d peerLen %u port %u peer0 %02x%02x",
                     hardware ? "hardware" : "software", static_cast<int>(from.ss_family),
                     static_cast<unsigned>(peerLen), static_cast<unsigned>(peerPort), peer[0],
                     peer[1]);
        }

        struct timespec received = {};
        /* Convert the counter stamp to served time by reading both now. */
        int64_t toServed = 0;
        if (hardware) {
            struct timespec servedNow = {};
            m_ref->now(servedNow);
            toServed = static_cast<int64_t>(servedNow.tv_sec) * 1000000000LL + servedNow.tv_nsec -
                       ptp::nowNs();
            const int64_t t2 = hwRx.ns + toServed;
            received.tv_sec  = static_cast<time_t>(t2 / 1000000000);
            received.tv_nsec = static_cast<long>(t2 % 1000000000);
        } else {
            m_ref->now(received);
        }

        const uint32_t nowMs = pdTICKS_TO_MS(xTaskGetTickCount());

        if (len < 0) {
            /* A timeout is normal. */
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGW(TAG, "recvfrom: errno %d", errno);
            }
        } else if ((packet[0] & 0x07) == kModeControl &&
                   limiter.check(from, nowMs) != Action::Answer) {
            /* Mode 6 is rate limited too (a small request gets a large answer);
             * over the limit it is dropped, since ntpq does not expect a KoD. */
            ++dropped;
            ++winDropped;
            m_dropped.store(dropped, std::memory_order_relaxed);
        } else if ((packet[0] & 0x07) == kModeControl) {
            const TimeReference ref = m_ref->reference();
            ControlFacts        facts;
            fillControlFacts(facts, ref);
            m_control.fetch_add(1, std::memory_order_relaxed);

            /* Fragments are sent back to back; ntpq gives up on holes. */
            uint8_t  control[kControlReplyBytes] = {};
            uint16_t total                       = 0;
            uint16_t offset                      = 0;
            for (int fragment = 0; fragment < 8; ++fragment) {
                const size_t controlLen =
                    buildControlReply(control, sizeof(control), packet, static_cast<size_t>(len),
                                      facts, offset, total);
                if (controlLen == 0 ||
                    sendto(sock, control, controlLen, 0, reinterpret_cast<const sockaddr *>(&from),
                           fromLen) < 0) {
                    if (controlLen != 0) {
                        ESP_LOGW(TAG, "sendto (control): errno %d", errno);
                    }
                    break;
                }
                offset =
                    static_cast<uint16_t>(offset + std::min<size_t>(kControlChunk, total - offset));
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

                    /* For ServePath: the client's send time is the request's
                     * transmit field. */
                    if (!kod) {
                        recordServePath(from, received, req.transmit);
                    }

                    const TimeReference ref         = m_ref->reference();
                    struct timespec     transmitted = {};
                    m_ref->now(transmitted);
                    /* On the sendto() path, move the transmit stamp later by
                     * the measured send delay. Not on the raw path, which is
                     * much faster than sendto(). */
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

                    uint8_t savedTransmit[kStampBytes] = {};
                    bool    interleavedMode            = false;
                    /* A zero origin, or receive equal to transmit, is a basic
                     * request. */
                    const bool mayInterleave =
                        !kod && !isZeroStamp(req.origin) &&
                        std::memcmp(req.receive, req.transmit, kStampBytes) != 0;
                    if (mayInterleave) {
                        m_ilTry.fetch_add(1, std::memory_order_relaxed);
                        interleavedMode = pairs.take(from, req.origin, savedTransmit, nowMs);
                        if (interleavedMode) {
                            m_ilHit.fetch_add(1, std::memory_order_relaxed);
                        }
                    }

                    uint8_t reply[kPacketBytes] = {};
                    if (interleavedMode) {
                        m_interleaved.fetch_add(1, std::memory_order_relaxed);
                    }
                    buildReply(reply, req, ref, kod, received, transmitted,
                               interleavedMode ? savedTransmit : nullptr);
                    /* Receive and transmit must differ in a basic reply, or the
                     * client cannot tell the modes apart (RFC 9769 section 2). */
                    if (!interleavedMode &&
                        std::memcmp(receiveStamp, liveStamp, kStampBytes) == 0) {
                        bumpFraction(liveStamp);
                        std::memcpy(reply + kTransmitOffset, liveStamp, kStampBytes);
                    }

                    /* Raw frame when the request was hardware stamped. Its own
                     * transmit field is the software stamp taken before the
                     * send (uncorrected on this path); the hardware stamp is
                     * saved for an interleaved client's next request. */
                    int64_t       measuredTx                 = 0;
                    uint8_t       measuredStamp[kStampBytes] = {};
                    const int64_t replySent                  = esp_timer_get_time();
                    int           replyRc                    = -1;
                    if (kRawSend && hardware &&
                        ptp::sendUdp(peer, peerLen, peerPort, hwRx.mac, reply, sizeof(reply),
                                     mayInterleave ? &measuredTx : nullptr)) {
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
                        if (measuredTx != 0) {
                            const int64_t   servedTx = measuredTx + toServed;
                            struct timespec measured = {};
                            measured.tv_sec          = static_cast<time_t>(servedTx / 1000000000);
                            measured.tv_nsec         = static_cast<long>(servedTx % 1000000000);
                            writeTimestamp(measuredStamp, measured);
                        }
                        pairs.save(from, receiveStamp, measuredTx != 0 ? measuredStamp : liveStamp,
                                   nowMs);
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
            /* Publish the completed window's rate before clearing it. */
            m_ratePerMin.store((winAnswered * 60000u) / kReportEveryMs, std::memory_order_relaxed);
            if (winAnswered != 0 || winKod != 0 || winDropped != 0) {
                ESP_LOGD(TAG, "answered %u, KoD %u, dropped %u in the last %u s", winAnswered,
                         winKod, winDropped, kReportEveryMs / 1000);
            }
            m_ilSaved.store(pairs.m_saved, std::memory_order_relaxed);
            m_ilKeyOnly.store(pairs.m_taken, std::memory_order_relaxed);
            winAnswered = winKod = winDropped = 0;
            reportAt                          = nowMs + kReportEveryMs;
        }
    }

    /* applySettings() closes the socket once the handle is cleared. */
    m_task = nullptr;
    ESP_LOGI(TAG, "responder stopped");
    vTaskDelete(nullptr);
}
