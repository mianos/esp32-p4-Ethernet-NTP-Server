#include "tx_lag.h"

#include <cerrno>
#include <cstring>

#include <sys/socket.h>
#include <unistd.h>

#include <netinet/in.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

#include "net_state.h"

namespace {

constexpr const char *TAG = "tx_lag";

/* How often to measure. The figure is a property of the build and the link, not
 * of the moment, so this only has to track a slow change; the EWMA does the
 * rest. */
constexpr uint32_t kEveryMs = 30000;

/* Sent and read back. Twelve bytes: big enough that the stack does the real
 * work, small enough that the wire time (about 1 us at 100 Mbit) is well under
 * what this is measuring. */
constexpr size_t kProbeBytes = 12;

/* A half round trip outside this range is not a transmit path. The floor is
 * about the granularity of the clock; the ceiling is a busy moment. */
constexpr int64_t kMinNs = 2000;
constexpr int64_t kMaxNs = 5000000;

/* Weight of a new sample. A handful of probes converge and a single outlier
 * cannot move the figure much. */
constexpr double kAlpha = 0.25;

/* How long one probe waits before giving up. */
constexpr int kProbeTimeoutMs = 20;

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
            const int64_t previous = m_lagNs.load(std::memory_order_relaxed);
            const int64_t next =
                previous == 0 ? lag
                              : static_cast<int64_t>(previous + kAlpha * (lag - previous));
            m_lagNs.store(next, std::memory_order_relaxed);
            const uint32_t n = m_samples.fetch_add(1, std::memory_order_relaxed) + 1;
            if (n == 1 || n % 20 == 0) {
                ESP_LOGI(TAG, "transmit correction %.1f us over %u probes", lagUs(), n);
            }
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

    /* Our own address, so the probe traverses the same stack the reply does.
     * lwIP short-circuits a self-send, which is why this covers the stack and
     * not the PHY. */
    esp_netif_ip_info_t ip = {};
    if (esp_netif_get_ip_info(netif, &ip) != ESP_OK || ip.ip.addr == 0) {
        return -1;
    }

    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        return -1;
    }

    struct sockaddr_in to    = {};
    to.sin_family            = AF_INET;
    to.sin_port              = htons(0); /* filled in after the bind */
    to.sin_addr.s_addr       = ip.ip.addr;
    if (bind(sock, reinterpret_cast<const sockaddr *>(&to), sizeof(to)) != 0) {
        close(sock);
        return -1;
    }

    socklen_t len = sizeof(to);
    if (getsockname(sock, reinterpret_cast<sockaddr *>(&to), &len) != 0) {
        close(sock);
        return -1;
    }

    struct timeval tv = {};
    tv.tv_sec         = 0;
    tv.tv_usec        = kProbeTimeoutMs * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t payload[kProbeBytes] = {};
    int64_t result               = -1;

    const int64_t sent = esp_timer_get_time();
    if (sendto(sock, payload, sizeof(payload), 0, reinterpret_cast<const sockaddr *>(&to),
               sizeof(to)) >= 0) {
        uint8_t back[kProbeBytes];
        if (recvfrom(sock, back, sizeof(back), 0, nullptr, nullptr) ==
            static_cast<int>(sizeof(payload))) {
            const int64_t roundTrip = esp_timer_get_time() - sent;
            /* Half of it is one direction. */
            result = roundTrip * 1000 / 2;
        }
    }

    close(sock);
    return result;
}
