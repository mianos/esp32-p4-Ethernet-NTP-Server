#include "ntp_server.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>

#include "esp_log.h"
#include "lwip/inet.h"
#include "lwip/ip6_addr.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

#include "ntp_time.h"
#include "settings.h"

namespace {

/* ---- Shape of the conversation ---------------------------------------- */

constexpr int          kPort             = 123;
constexpr size_t       kPacketBytes      = 48;   /* NTPv4 header, no MAC */
constexpr uint32_t     kReceiveTimeoutMs = 1000; /* how often the task looks up */
constexpr uint32_t     kReportEveryMs    = 60000;
constexpr int          kTaskStack        = 4096;
constexpr UBaseType_t  kTaskPriority     = 5;    /* above the status logger, below lwIP */

constexpr size_t       kStampBytes    = 8;
constexpr size_t       kOriginOffset  = 24;
constexpr size_t       kReceiveOffset = 32;
constexpr size_t       kTransmitOffset = 40;

/* ---- Protocol constants ----------------------------------------------- */

/* Seconds between 1900-01-01 and 1970-01-01. */
constexpr uint32_t kNtpUnixDelta = 2208988800u;

constexpr uint8_t kModeClient = 3; /* the only mode this device answers */
constexpr uint8_t kModeServer = 4;
constexpr uint8_t kVersionOut = 4;

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

constexpr uint32_t kRefIdInit = (static_cast<uint32_t>('I') << 24) | ('N' << 16) | ('I' << 8) | 'T';
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

/* How recent the last sync has to be for the clock to count as locked. The
 * upstream polls far more often than this; the window only has to catch the
 * case where discipline has stopped altogether. */
constexpr time_t kLockWindowS = 3600;

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

uint32_t rootDispersion(const TimeReference &ref, time_t now)
{
    double seconds = static_cast<double>(kRootDelay) / 65536.0;
    if (ref.lastSync != 0 && now > ref.lastSync) {
        seconds += static_cast<double>(now - ref.lastSync) * kDispersionPpm * 1e-6;
    }
    const double fixed = seconds * 65536.0;
    return fixed >= 65535.0 ? 0xFFFFu : static_cast<uint32_t>(fixed);
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

/* "131.84.1.95:123" or "[2401:...]:123", for the log only. */
std::string sourceText(const sockaddr_storage &src)
{
    char text[64] = {};
    if (src.ss_family == AF_INET) {
        const auto    *v4 = reinterpret_cast<const sockaddr_in *>(&src);
        const uint8_t *b  = reinterpret_cast<const uint8_t *>(&v4->sin_addr);
        std::snprintf(text, sizeof(text), "%u.%u.%u.%u:%u", b[0], b[1], b[2], b[3],
                      ntohs(v4->sin_port));
#if CONFIG_LWIP_IPV6
    } else {
        const auto *v6 = reinterpret_cast<const sockaddr_in6 *>(&src);
        ip6_addr_t  lwip{};
        std::memcpy(lwip.addr, &v6->sin6_addr, sizeof(lwip.addr));
        char addr[IP6ADDR_STRLEN_MAX] = {};
        ip6addr_ntoa_r(&lwip, addr, sizeof(addr));
        std::snprintf(text, sizeof(text), "[%s]:%u", addr, ntohs(v6->sin6_port));
#endif
    }
    return text;
}

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
    uint8_t transmit[kStampBytes] = {};
};

/* Worth answering at all? Long enough to hold a header, a version both ends
 * speak, and in client mode. Replying to a mode 4 packet is how you build an
 * infinite conversation between two servers, and mode 6/7 is control traffic
 * this device has no keys for, so both are silence. Extension fields past byte
 * 48 are ignored and never echoed. */
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
    std::memcpy(req.transmit, packet + kTransmitOffset, kStampBytes);
    return true;
}

/* The reply is always exactly 48 bytes, whatever arrived: an NTP server that
 * sends more than it receives is a reflector, and 48 is all a client needs to
 * compute offset and delay. */
void buildReply(uint8_t *out, const Request &req, const TimeReference &ref, bool kod,
                const struct timespec &received, const struct timespec &transmitted)
{
    const uint8_t  stratum = kod ? 0 : ref.stratum;
    const uint8_t  leap    = (kod || ref.locked) ? kLeapNoWarning : kLeapAlarm;
    const uint32_t refId   = kod ? kRefIdRate : ref.refId;

    out[0] = static_cast<uint8_t>((leap << 6) | (kVersionOut << 3) | kModeServer);
    out[1] = stratum;
    out[2] = req.poll;
    out[3] = kPrecision;
    store32(out + 4, kRootDelay);
    store32(out + 8, rootDispersion(ref, received.tv_sec));
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
     * which some do on the first packet. */
    std::memcpy(out + kOriginOffset, req.transmit, kStampBytes);
    writeTimestamp(out + kReceiveOffset, received);
    writeTimestamp(out + kTransmitOffset, transmitted);
}

/* The upstream SNTP clock, dressed as a TimeRef. */
class UpstreamTimeRef : public TimeRef {
public:
    TimeReference reference() const override
    {
        NtpTime &ntp = NtpTime::instance();
        const time_t last = ntp.lastSync();
        const time_t now  = time(nullptr);

        TimeReference ref;
        ref.lastSync = last;
        /* "Synced" only records ever having been set, so the freshness window
         * is what makes this a lock: an upstream that went away an hour ago
         * leaves a clock that is drifting with no one correcting it. The
         * client's own reach register is deliberately not consulted -- it
         * flaps on a single lost poll, and it is empty for a clock set by
         * POST /ntp/sync while timeSource is off. */
        ref.locked = ntp.synced() && last != 0 && now > last && (now - last) < kLockWindowS;
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
        char     tail = 0;
        if (std::sscanf(text.c_str(), "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) != 4) {
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

std::unique_ptr<TimeRef> makeUpstreamTimeRef()
{
    return std::unique_ptr<TimeRef>(new UpstreamTimeRef());
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
        m_ref = makeUpstreamTimeRef();
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
     * quick off/on toggle cannot end up with two of them. */
    for (int i = 0; i < 30 && m_task != nullptr; ++i) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return ESP_OK;
}

TimeReference NtpServer::reference() const
{
    return m_ref ? m_ref->reference() : TimeReference{};
}

void NtpServer::serveLoop()
{
    RateLimiter limiter;
    uint32_t    answered = 0, kodSent = 0, dropped = 0;      /* cumulative */
    uint32_t    winAnswered = 0, winKod = 0, winDropped = 0; /* since report */
    uint32_t    reportAt    = pdTICKS_TO_MS(xTaskGetTickCount()) + kReportEveryMs;
    bool        firstSeen   = false;

    while (m_running.load(std::memory_order_relaxed)) {
        sockaddr_storage from    = {};
        socklen_t        fromLen = sizeof(from);

        /* Room for extension fields, so a long request is still parsed from its
         * first 48 bytes instead of being dropped as unrecognised. */
        uint8_t   packet[kPacketBytes * 2] = {};
        const int len = recvfrom(m_sock, packet, sizeof(packet), 0,
                                 reinterpret_cast<sockaddr *>(&from), &fromLen);

        /* lwIP has no SO_TIMESTAMP, so this read after the syscall is the
         * receive timestamp: scheduler latency included, hardware stamping
         * not. That is the accuracy this phase can honestly claim. */
        struct timespec received = {};
        m_ref->now(received);

        const uint32_t nowMs = pdTICKS_TO_MS(xTaskGetTickCount());

        if (len < 0) {
            /* The 1 s timeout is the normal way round this loop. */
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGW(TAG, "recvfrom: errno %d", errno);
            }
        } else {
            Request req;
            if (!parseRequest(packet, len, req)) {
                ++dropped;
                ++winDropped;
            } else {
                const Action action = limiter.check(from, nowMs);
                if (action == Action::Drop) {
                    ++dropped;
                    ++winDropped;
                } else {
                    const bool kod = action == Action::Kod;

                    /* Take both stamps around the reply build, which is as
                     * close to the wire as a software timestamp gets. */
                    const TimeReference ref         = m_ref->reference();
                    struct timespec    transmitted = {};
                    m_ref->now(transmitted);

                    uint8_t reply[kPacketBytes] = {};
                    buildReply(reply, req, ref, kod, received, transmitted);

                    if (sendto(m_sock, reply, sizeof(reply), 0,
                               reinterpret_cast<const sockaddr *>(&from), fromLen) < 0) {
                        ESP_LOGW(TAG, "sendto: errno %d", errno);
                    } else if (kod) {
                        ++kodSent;
                        ++winKod;
                    } else {
                        ++answered;
                        ++winAnswered;
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
            if (winAnswered != 0 || winKod != 0 || winDropped != 0) {
                /* One line a minute at most: the serial console is the only
                 * local debug there is, and a LAN polling at minpoll would
                 * otherwise bury everything else in the log. */
                ESP_LOGI(TAG, "answered %u, KoD %u, dropped %u in the last %u s",
                         winAnswered, winKod, winDropped, kReportEveryMs / 1000);
            }
            winAnswered = winKod = winDropped = 0;
            reportAt = nowMs + kReportEveryMs;
        }
    }

    close(m_sock);
    m_sock = -1;
    m_task = nullptr;
    ESP_LOGI(TAG, "responder stopped");
    vTaskDelete(nullptr);
}
