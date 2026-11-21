#include "ntp/tx_lag.h"

#include <algorithm>
#include <cerrno>
#include <cstring>

#include <sys/socket.h>
#include <unistd.h>

#include <netinet/in.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

#include "net/net_state.h"

namespace {

constexpr const char *TAG = "tx_lag";

/* How often to measure. The figure is a property of the moment, not of the
 * build, so this is frequent enough that the window closes faster than the
 * thing it is tracking moves, and cheap enough to ignore: one datagram to
 * ourselves every few seconds. */
constexpr uint32_t kEveryMs = 5000;

/* How many samples before the centre is published. Five is enough for a median
 * to survive two bad ones; below it, the value would jump between the first few
 * probes and a reply would carry whatever the last one saw. */
constexpr uint32_t kMinSamples = 5;

/* Twelve bytes: big enough that the stack does the real work, small enough that
 * the wire time (about 1 us at 100 Mbit) is well under what this measures. */
constexpr size_t kProbeBytes = 12;

/* A half round trip outside this range is not a transmit path. The floor is
 * about the granularity of the clock; the ceiling is a busy moment. */
constexpr int64_t kMinNs = 2000;
constexpr int64_t kMaxNs = 5000000;

/* How long one probe waits before giving up. */
/* The probe socket's own port, so the datagram is delivered to a bound socket the
 * way a reply is. Sending to a dead port instead works, but the ICMP that comes
 * back is handled inside the same call and inflates the figure. */
constexpr uint16_t kProbePort = 0; /* 0 = let lwIP pick */

constexpr int kTaskStack  = 4096;
constexpr int kTaskPrio   = 4; /* below the responder and the status logger */

}  // namespace

TxLag &TxLag::instance()
{
    static TxLag lag;
    return lag;
}

esp_err_t TxLag::begin()
{
    if (m_task != nullptr) {
        return ESP_OK;
    }
    if (xTaskCreate(&TxLag::taskEntry, "tx_lag", kTaskStack, this, kTaskPrio, &m_task) != pdPASS) {
        m_task = nullptr;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void TxLag::taskEntry(void *arg)
{
    static_cast<TxLag *>(arg)->task();
}

void TxLag::absorb(int64_t lag)
{
    m_ring[m_ringPos] = lag;
    m_ringPos         = (m_ringPos + 1) % kRing;
    if (m_ringFill < kRing) {
        ++m_ringFill;
    }
    const uint32_t n = m_samples.fetch_add(1, std::memory_order_relaxed) + 1;

    /* A median needs enough of the ring to be a median of anything. Until then
     * the newest sample stands, which is what the old code did on its first. */
    if (m_ringFill < kMinSamples) {
        m_lagNs.store(lag, std::memory_order_relaxed);
        return;
    }

    /* Insertion sort of at most fifteen values: cheaper than a call and the ring
     * is already in cache. */
    int64_t sorted[kRing] = {};
    for (uint32_t i = 0; i < m_ringFill; ++i) {
        const int64_t v = m_ring[i];
        uint32_t      j = i;
        while (j > 0 && sorted[j - 1] > v) {
            sorted[j] = sorted[j - 1];
            --j;
        }
        sorted[j] = v;
    }

    const int64_t median = sorted[m_ringFill / 2];

    /* Spread around that centre, taken the same way so a single stall cannot
     * widen the stated error. */
    int64_t deviations[kRing] = {};
    for (uint32_t i = 0; i < m_ringFill; ++i) {
        const int64_t d = m_ring[i] - median;
        deviations[i]   = d < 0 ? -d : d;
    }
    for (uint32_t i = 0; i < m_ringFill; ++i) {
        const int64_t v = deviations[i];
        uint32_t      j = i;
        while (j > 0 && deviations[j - 1] > v) {
            deviations[j] = deviations[j - 1];
            --j;
        }
        deviations[j] = v;
    }

    m_lagNs.store(median, std::memory_order_relaxed);
    m_scatterNs.store(deviations[m_ringFill / 2], std::memory_order_relaxed);

    /* Every sample, because the point of this run is to see the path move: the
     * raw value says what one probe saw, the two aggregates say what the
     * correction and the claimed error would be if they were published now. */
    ESP_LOGI(TAG, "txprobe raw %lld us  median %.1f  mad %.1f  n %u", (long long)(lag / 1000),
             lagUs(), scatterUs(), (unsigned)n);
}

void TxLag::recordReply(int64_t ns)
{
    const uint32_t pos = m_replyPos.fetch_add(1, std::memory_order_relaxed);
    m_reply[pos % kReplyRing] = ns;
    if (pos >= kReplyRing) {
        m_replyFlooded.fetch_add(1, std::memory_order_relaxed);
    }
}

void TxLag::reportReplies()
{
    /* Drained by the same task that probes, once a minute. A slot that arrives
     * while this runs is simply counted next time. */
    const uint32_t filled = std::min<uint32_t>(m_replyPos.exchange(0, std::memory_order_relaxed),
                                               kReplyRing);
    if (filled < 4) {
        return;
    }
    const uint32_t flooded = m_replyFlooded.exchange(0, std::memory_order_relaxed);

    int64_t sorted[kReplyRing] = {};
    for (uint32_t i = 0; i < filled; ++i) {
        const int64_t v = m_reply[i];
        uint32_t      j = i;
        while (j > 0 && sorted[j - 1] > v) {
            sorted[j] = sorted[j - 1];
            --j;
        }
        sorted[j] = v;
    }

    const auto pct = [&](double p) { return sorted[static_cast<uint32_t>(p * (filled - 1))]; };
    ESP_LOGI(TAG, "replypath n %u p10 %.1f med %.1f p90 %.1f us%s", (unsigned)filled,
             pct(0.10) / 1000.0, pct(0.50) / 1000.0, pct(0.90) / 1000.0,
             flooded > 0 ? " (flooded)" : "");
}

void TxLag::task()
{
    /* Nothing to measure until there is an address to send to. */
    for (;;) {
        if (net_state::linkUp() && net_state::netif() != nullptr) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    for (;;) {
        const int64_t lag = probe();
        if (lag >= kMinNs && lag <= kMaxNs) {
            absorb(lag);
        }
        /* Every twelfth probe: a minute, matching the reply report cadence. */
        if (++m_tick >= 12) {
            m_tick = 0;
            reportReplies();
        }
        vTaskDelay(pdMS_TO_TICKS(kEveryMs));
    }
}

int64_t TxLag::probe()
{
    esp_netif_t *netif = net_state::netif();
    if (netif == nullptr) {
        return -1;
    }

    /* Our own address and our own bound port, so the datagram traverses the
     * stack and the driver the way a reply does, and is delivered rather than
     * refused. Whether it is read does not matter: the interval ends when the
     * call returns. */
    esp_netif_ip_info_t ip = {};
    if (esp_netif_get_ip_info(netif, &ip) != ESP_OK || ip.ip.addr == 0) {
        return -1;
    }

    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        return -1;
    }

    struct sockaddr_in to = {};
    to.sin_family         = AF_INET;
    to.sin_port           = htons(kProbePort);
    to.sin_addr.s_addr    = ip.ip.addr;
    if (bind(sock, reinterpret_cast<const sockaddr *>(&to), sizeof(to)) != 0) {
        close(sock);
        return -1;
    }
    socklen_t toLen = sizeof(to);
    if (getsockname(sock, reinterpret_cast<sockaddr *>(&to), &toLen) != 0) {
        close(sock);
        return -1;
    }

    uint8_t  payload[kProbeBytes] = {};
    int64_t result = -1;

    /* The quantity that matters is how long *after the stamp* the datagram is
     * still inside the stack, because that is how early the stamp is. sendto()
     * returning is where the userspace part of that ends, so its duration is a
     * direct measurement of it -- and a lower bound on the whole path, since the
     * driver and the wire happen after the call returns.
     *
     * This replaced a self-send and a half round trip. That one read about 1.5x
     * too high: it measured two stack traversals, two wakeups and the loopback
     * dispatch, where a reply does one. It showed up as a correction that moved
     * a client's offset from -39 us to +18 us instead of to about zero. */
    const int64_t sent = esp_timer_get_time();
    if (sendto(sock, payload, sizeof(payload), 0, reinterpret_cast<const sockaddr *>(&to),
               sizeof(to)) >= 0) {
        result = (esp_timer_get_time() - sent) * 1000;
    }

    close(sock);
    return result;
}
