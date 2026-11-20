#include "ptp.h"

#include "esp_attr.h"
#include "esp_eth_mac_esp.h"
#include "esp_log.h"

#include <sys/time.h>

#include <cstdlib>
#include <cstring>

namespace {

constexpr const char *TAG = "ptp";

/* Enough for the NTP excursions of a burst plus a PTP exchange's worth; the
 * responder takes its entry immediately after recvfrom(), so entries are only
 * briefly live. */
constexpr size_t kRing = 16;
constexpr int64_t kStampAgeNs = 500000000LL; /* an entry older than this is noise */

struct RxStamp {
    uint8_t  addr[16];
    uint8_t  addrLen;
    uint16_t port;
    bool     used;
    int64_t  ns;
};

esp_eth_handle_t s_eth = nullptr;
esp_eth_mac_t   *s_mac = nullptr;
bool             s_attached = false;
bool             s_enabled  = false;

} // namespace

namespace ptp {
volatile bool gHardwareOn = false;
}

namespace {

portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
RxStamp      s_ring[kRing];
size_t       s_next = 0;
ptp::Stats    s_stats{};

uint16_t be16(const uint8_t *p)
{
    return static_cast<uint16_t>((p[0] << 8) | p[1]);
}

void recordRx(const uint8_t *addr, uint8_t addrLen, uint16_t port, int64_t ns)
{
    portENTER_CRITICAL(&s_mux);
    RxStamp &slot = s_ring[s_next];
    s_next        = (s_next + 1) % kRing;
    std::memcpy(slot.addr, addr, addrLen);
    slot.addrLen = addrLen;
    slot.port    = port;
    slot.used    = true;
    slot.ns      = ns;
    portEXIT_CRITICAL(&s_mux);

    ++s_stats.rxStamps;
    s_stats.lastNs = ns;
}

/* A UDP datagram bound for port 123, in either family. Returns the source once
 * the frame has been checked far enough to have one. */
void maybeRecordUdp(const uint8_t *frame, uint32_t len, int64_t ns)
{
    if (len < 34) {
        return;
    }
    const uint16_t type = be16(frame + 12);
    if (type == 0x0800) {
        if (len < 14 + 20 + 8) {
            return;
        }
        const uint8_t ihl = static_cast<uint8_t>((frame[14] & 0x0f) * 4);
        if (ihl < 20 || len < static_cast<uint32_t>(14 + ihl + 8)) {
            return;
        }
        const uint8_t *udp = frame + 14 + ihl;
        if (frame[23] != 17 || be16(udp + 2) != 123) {
            return;
        }
        recordRx(frame + 26, 4, be16(udp), ns);
    } else if (type == 0x86dd) {
        if (len < 14 + 40 + 8 || frame[20] != 17) {
            return;
        }
        const uint8_t *udp = frame + 54;
        if (be16(udp + 2) != 123) {
            return;
        }
        recordRx(frame + 22, 16, be16(udp), ns);
    }
}

esp_err_t inputHandler(esp_eth_handle_t, uint8_t *buffer, uint32_t length, void *priv, void *info)
{
    const auto *stamp = static_cast<const eth_mac_time_t *>(info);
    const bool  valid = stamp != nullptr && stamp->seconds != 0;
    const int64_t ns  = valid ? static_cast<int64_t>(stamp->seconds) * 1000000000LL +
                                   static_cast<int64_t>(stamp->nanoseconds)
                              : 0;

    if (length >= 14 && be16(buffer + 12) == 0x88F7) {
        ++s_stats.frames;
        if (valid) {
            s_stats.lastNs = ns;
        }
        /* Nothing reads these yet; the frame is dropped rather than queued so
         * the buffer can go back the moment it is parsed. */
        free(buffer);
        return ESP_OK;
    }
    if (valid) {
        maybeRecordUdp(buffer, length, ns);
    }
    /* priv is the netif: esp_eth_update_input_path_info() overwrites the glue's
     * own priv slot with it, so it cannot be anything else. */
    return esp_netif_receive(static_cast<esp_netif_t *>(priv), buffer, length, NULL);
}

} // namespace

namespace ptp {

esp_err_t attach(esp_eth_handle_t eth, esp_eth_mac_t *mac, esp_netif_t *netif)
{
    if (s_attached) {
        return ESP_OK;
    }
    if (eth == nullptr || mac == nullptr || netif == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    s_eth = eth;
    s_mac = mac;
    const esp_err_t err = esp_eth_update_input_path_info(eth, inputHandler, netif);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "input path: %s", esp_err_to_name(err));
        return err;
    }
    s_attached = true;
    return ESP_OK;
}

esp_err_t enable(bool restart)
{
    if (s_mac == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_enabled && !restart) {
        return ESP_OK;
    }

    eth_mac_ptp_config_t config = {};
    /* The crystal, at 40 MHz: 25 ns is the finest the counter can be stepped and
     * is also what its increment is set to. The 80 MHz PLL is the alternative
     * and buys 12.5 ns for a clock that has to be kept running. */
    config.clk_src             = EMAC_PTP_CLK_SRC_XTAL;
    config.clk_src_period_ns   = 25.0f;
    config.required_accuracy_ns = 25.0f;
    config.roll_type           = ETH_PTP_DIGITAL_ROLLOVER;

    esp_err_t err = esp_eth_mac_ptp_enable(s_mac, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "phc enable: %s", esp_err_to_name(err));
        return err;
    }
    /* Every frame, not only PTP's: the responder needs the arrival stamp of a
     * UDP/123 datagram long before there is a PTP message to serve. */
    err = esp_eth_mac_enable_ts4all(s_mac, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ts4all: %s", esp_err_to_name(err));
        return err;
    }

    /* Start it at CLOCK_REALTIME so a stamp is an epoch instant and can be read
     * by eye. Before the clock is set this is 1970, which the anchor corrects
     * from the first fix. */
    struct timeval tv = {};
    gettimeofday(&tv, nullptr);
    eth_mac_time_t start = {};
    start.seconds        = static_cast<uint32_t>(tv.tv_sec);
    start.nanoseconds    = static_cast<uint32_t>(tv.tv_usec) * 1000;
    err = esp_eth_mac_set_ptp_time(s_mac, &start);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "phc set: %s", esp_err_to_name(err));
    }

    s_enabled     = true;
    ptp::gHardwareOn = true;
    ESP_LOGI(TAG, "phc running, 25 ns steps, started from %u.%09u",
             static_cast<unsigned>(start.seconds), static_cast<unsigned>(start.nanoseconds));
    return ESP_OK;
}

bool enabled()
{
    return s_enabled;
}

esp_err_t transmit(const void *frame, size_t len, int64_t *stampNs)
{
    if (stampNs != nullptr) {
        *stampNs = 0;
    }
    if (s_eth == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    eth_mac_time_t stamp = {};
    const esp_err_t err  = esp_eth_transmit_ctrl_vargs(s_eth, &stamp, 2, frame, len);
    if (err != ESP_OK) {
        return err;
    }
    if (stamp.seconds == 0) {
        ++s_stats.txMissing;
        return ESP_OK;
    }
    ++s_stats.txStamps;
    s_stats.lastNs = static_cast<int64_t>(stamp.seconds) * 1000000000LL +
                     static_cast<int64_t>(stamp.nanoseconds);
    if (stampNs != nullptr) {
        *stampNs = s_stats.lastNs;
    }
    return ESP_OK;
}

bool takeRxStamp(const uint8_t *addr, size_t addrLen, uint16_t port, int64_t *ns)
{
    if (addr == nullptr || ns == nullptr || addrLen > 16) {
        return false;
    }
    const int64_t now  = nowNs();
    bool          found = false;
    portENTER_CRITICAL(&s_mux);
    for (size_t i = 0; i < kRing; ++i) {
        RxStamp &slot = s_ring[i];
        if (!slot.used || slot.port != port || slot.addrLen != addrLen ||
            std::memcmp(slot.addr, addr, addrLen) != 0) {
            continue;
        }
        const int64_t age = now - slot.ns;
        if (age < 0 || age > kStampAgeNs) {
            slot.used = false;
            continue;
        }
        *ns       = slot.ns;
        slot.used = false;
        found     = true;
        break;
    }
    portEXIT_CRITICAL(&s_mux);
    return found;
}

Stats stats()
{
    return s_stats;
}

} // namespace ptp
