/* Compiles this file's ESP_LOGD lines in; they stay hidden until raised with
 * the console's `log_level debug <tag>`. */
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

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
#include "app/cores.h"

namespace {

constexpr const char *TAG = "tx_lag";

constexpr uint32_t kEveryMs = 5000;

/* Samples before the median is used; until then the newest sample is. */
constexpr uint32_t kMinSamples = 5;

constexpr size_t kProbeBytes = 12;

/* Probe results outside this range are discarded. */
constexpr int64_t kMinNs = 2000;
constexpr int64_t kMaxNs = 5000000;

/* The probe sends to its own bound socket: a dead port's ICMP reply would be
 * handled inside the call and inflate it. */
constexpr uint16_t kProbePort = 0; /* 0 = let lwIP pick */

constexpr int kTaskStack = 4096;
constexpr int kTaskPrio  = 4; /* below the responder and the status logger */

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
    if (xTaskCreatePinnedToCore(&TxLag::taskEntry, "tx_lag", kTaskStack, this, kTaskPrio, &m_task,
                                cores::kWork) != pdPASS) {
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

    if (m_ringFill < kMinSamples) {
        m_lagNs.store(lag, std::memory_order_relaxed);
        return;
    }

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

    /* Median absolute deviation, so one stall does not widen it. */
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

    ESP_LOGD(TAG, "txprobe raw %lld us  median %.1f  mad %.1f  n %u", (long long)(lag / 1000),
             lagUs(), scatterUs(), (unsigned)n);
}

void TxLag::recordReply(int64_t ns)
{
    const uint32_t pos        = m_replyPos.fetch_add(1, std::memory_order_relaxed);
    m_reply[pos % kReplyRing] = ns;
    if (pos >= kReplyRing) {
        m_replyFlooded.fetch_add(1, std::memory_order_relaxed);
    }
}

void TxLag::reportReplies()
{
    /* Called by the probe task once a minute. */
    const uint32_t filled =
        std::min<uint32_t>(m_replyPos.exchange(0, std::memory_order_relaxed), kReplyRing);
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
    ESP_LOGD(TAG, "replypath n %u p10 %.1f med %.1f p90 %.1f us%s", (unsigned)filled,
             pct(0.10) / 1000.0, pct(0.50) / 1000.0, pct(0.90) / 1000.0,
             flooded > 0 ? " (flooded)" : "");
}

void TxLag::task()
{
    /* Wait for an address. */
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
        /* Every twelfth probe, about a minute. */
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

    /* Our own address and a bound port, so the datagram is delivered rather
     * than refused; it is never read. */
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

    uint8_t payload[kProbeBytes] = {};
    int64_t result               = -1;

    /* sendto()'s duration: how long after a pre-send stamp the datagram is
     * still in the stack. A half round trip read about 1.5x too high. */
    const int64_t sent = esp_timer_get_time();
    if (sendto(sock, payload, sizeof(payload), 0, reinterpret_cast<const sockaddr *>(&to),
               sizeof(to)) >= 0) {
        result = (esp_timer_get_time() - sent) * 1000;
    }

    close(sock);
    return result;
}
