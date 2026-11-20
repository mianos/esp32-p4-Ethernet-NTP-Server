#include "ptp.h"

#include "esp_attr.h"
#include "esp_eth_mac_esp.h"
#include "esp_timer.h"
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
    uint8_t  mac[6];
    uint16_t port;
    bool     used;
    int64_t  ns;
};

esp_eth_handle_t s_eth   = nullptr;
esp_eth_mac_t   *s_mac   = nullptr;
esp_netif_t     *s_netif = nullptr;
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
uint8_t       s_lastFail = 0;

void fail(uint8_t reason)
{
    s_lastFail = reason;
    if (s_stats.txMissing == 0) {
        ESP_LOGW(TAG, "sendUdp refused: reason %u", static_cast<unsigned>(reason));
    }
}

uint16_t be16(const uint8_t *p)
{
    return static_cast<uint16_t>((p[0] << 8) | p[1]);
}

void recordRx(const uint8_t *mac, const uint8_t *addr, uint8_t addrLen, uint16_t port, int64_t ns)
{
    portENTER_CRITICAL(&s_mux);
    RxStamp &slot = s_ring[s_next];
    s_next        = (s_next + 1) % kRing;
    std::memcpy(slot.addr, addr, addrLen);
    std::memcpy(slot.mac, mac, 6);
    slot.addrLen = addrLen;
    slot.port    = port;
    slot.used    = true;
    slot.ns      = ns;
    portEXIT_CRITICAL(&s_mux);

    ++s_stats.rxStamps;
    s_stats.lastNs = ns;
    if (s_stats.rxStamps <= 20) {
        ESP_LOGW(TAG, "record port %u from %u.%u.%u.%u", static_cast<unsigned>(port), addr[0],
                 addr[1], addr[2], addr[3]);
    }
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
        recordRx(frame + 6, frame + 26, 4, be16(udp), ns);
    } else if (type == 0x86dd) {
        if (len < 14 + 40 + 8 || frame[20] != 17) {
            return;
        }
        const uint8_t *udp = frame + 54;
        if (be16(udp + 2) != 123) {
            return;
        }
        recordRx(frame + 6, frame + 22, 16, be16(udp), ns);
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
    s_eth   = eth;
    s_mac   = mac;
    s_netif = netif;
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
    config.clk_src            = EMAC_PTP_CLK_SRC_XTAL;
    config.clk_src_period_ns  = 25.0f;
    /* 40 and not the 25 ns the source ticks at: the driver derives the addend
     * from 2^32 * period / accuracy, and at 25/25 that is exactly 2^32, which
     * truncates to zero in a uint32. 40 ns keeps it well inside and still
     * resolves far finer than the microseconds this clock has to serve. */
    config.required_accuracy_ns = 40.0f;
    config.roll_type          = ETH_PTP_DIGITAL_ROLLOVER;

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

uint32_t sum16(const uint8_t *p, size_t n, uint32_t sum)
{
    while (n > 1) {
        sum += static_cast<uint32_t>((p[0] << 8) | p[1]);
        p += 2;
        n -= 2;
    }
    if (n != 0) {
        sum += static_cast<uint32_t>(p[0] << 8);
    }
    return sum;
}

uint16_t fold(uint32_t sum)
{
    while ((sum >> 16) != 0) {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    return static_cast<uint16_t>(~sum);
}

constexpr size_t kMaxUdpPayload = 64;

} // namespace

namespace ptp {

bool sendUdp(const uint8_t *dstAddr, size_t dstAddrLen, uint16_t dstPort, const uint8_t dstMac[6],
             const uint8_t *payload, size_t len, int64_t *stampNs)
{
    if (!s_enabled || s_netif == nullptr || s_eth == nullptr || dstAddr == nullptr ||
        dstMac == nullptr || payload == nullptr || len > kMaxUdpPayload) {
        fail(1);
        return false;
    }
    uint8_t ourMac[6] = {};
    if (esp_eth_ioctl(s_eth, ETH_CMD_G_MAC_ADDR, ourMac) != ESP_OK) {
        fail(2);
        return false;
    }

    constexpr size_t kEthLen = 14;
    /* Static, not on the stack: the responder task has a small stack and this
     * is one sender. */
    static uint8_t frame[kEthLen + 40 + 8 + kMaxUdpPayload];
    std::memset(frame, 0, sizeof(frame));
    std::memcpy(frame, dstMac, 6);
    std::memcpy(frame + 6, ourMac, 6);

    size_t   udpOff = 0;
    uint16_t udpLen = static_cast<uint16_t>(8 + len);
    if (dstAddrLen == 4) {
        esp_netif_ip_info_t info = {};
        if (esp_netif_get_ip_info(s_netif, &info) != ESP_OK || info.ip.addr == 0) {
            fail(3);
            return false;
        }
        const uint32_t src = info.ip.addr;
        frame[12]          = 0x08;
        frame[13]          = 0x00;
        uint8_t *ip        = frame + kEthLen;
        ip[0]              = 0x45;
        ip[2]              = static_cast<uint8_t>((20 + udpLen) >> 8);
        ip[3]              = static_cast<uint8_t>((20 + udpLen) & 0xff);
        ip[8]              = 64;
        ip[9]              = 17;
        std::memcpy(ip + 12, &src, 4);
        std::memcpy(ip + 16, dstAddr, 4);
        const uint16_t ipSum = fold(sum16(ip, 20, 0));
        ip[10]               = static_cast<uint8_t>(ipSum >> 8);
        ip[11]               = static_cast<uint8_t>(ipSum & 0xff);
        udpOff               = kEthLen + 20;
    } else if (dstAddrLen == 16) {
        /* LWIP_IPV6_NUM_ADDRESSES is three in this build, so the array covers
         * the netif and the returned count is bounded by it. */
        esp_ip6_addr_t addresses[3] = {};
        const int      count        = esp_netif_get_all_ip6(s_netif, addresses);
        const uint8_t *src          = nullptr;
        for (int i = 0; i < count; ++i) {
            const uint8_t *candidate = reinterpret_cast<const uint8_t *>(&addresses[i].addr);
            const bool linklocal = candidate[0] == 0xfe && (candidate[1] & 0xc0) == 0x80;
            const bool dstLinklocal = dstAddr[0] == 0xfe && (dstAddr[1] & 0xc0) == 0x80;
            if (linklocal == dstLinklocal) {
                src = candidate;
                break;
            }
        }
        if (src == nullptr) {
            fail(4);
            return false;
        }
        frame[12] = 0x86;
        frame[13] = 0xdd;
        uint8_t *ip = frame + kEthLen;
        ip[0]       = 0x60;
        ip[4]       = static_cast<uint8_t>(udpLen >> 8);
        ip[5]       = static_cast<uint8_t>(udpLen & 0xff);
        ip[6]       = 17;
        ip[7]       = 64;
        std::memcpy(ip + 8, src, 16);
        std::memcpy(ip + 24, dstAddr, 16);
        udpOff = kEthLen + 40;
    } else {
        fail(5);
        return false;
    }

    uint8_t *udp = frame + udpOff;
    udp[0]       = 0;
    udp[1]       = 123;
    udp[2]       = static_cast<uint8_t>(dstPort >> 8);
    udp[3]       = static_cast<uint8_t>(dstPort & 0xff);
    udp[4]       = static_cast<uint8_t>(udpLen >> 8);
    udp[5]       = static_cast<uint8_t>(udpLen & 0xff);
    std::memcpy(udp + 8, payload, len);

    /* IPv4 does not require a UDP checksum and a zero means "not computed";
     * IPv6 does, over a pseudo-header. */
    if (dstAddrLen == 16) {
        uint32_t sum = 0;
        sum          = sum16(frame + kEthLen + 8, 32, sum);
        sum += udpLen;
        sum += 17;
        sum = sum16(udp, udpLen, sum);
        const uint16_t c = fold(sum);
        udp[6]           = static_cast<uint8_t>(c >> 8);
        udp[7]           = static_cast<uint8_t>(c & 0xff);
    }

    const size_t total = udpOff + udpLen;
    const esp_err_t err = transmit(frame, total, stampNs);
    if (s_stats.txStamps == 0) {
        ESP_LOGI(TAG, "first raw send: %u bytes to %u.%u.%u.%u:%u, %s",
                 static_cast<unsigned>(total), dstAddr[0], dstAddr[1], dstAddr[2], dstAddr[3],
                 static_cast<unsigned>(dstPort), esp_err_to_name(err));
    }
    if (err != ESP_OK) {
        fail(6);
        ESP_LOGW(TAG, "transmit: %s", esp_err_to_name(err));
        return false;
    }
    if (stampNs == nullptr || *stampNs == 0) {
        fail(7);
        return false;
    }
    return true;
}

bool takeRx(const uint8_t *addr, size_t addrLen, uint16_t port, Rx *out)
{
    if (addr == nullptr || out == nullptr || addrLen > 16) {
        return false;
    }
    const int64_t now   = nowNs();
    bool          found = false;
    int64_t       age   = 0;
    /* No logging inside this section: the responder's log hook sends over
     * lwIP, and this runs with interrupts off. The first miss is reported
     * below, once the lock is released. */
    portENTER_CRITICAL(&s_mux);
    for (size_t i = 0; i < kRing; ++i) {
        RxStamp &slot = s_ring[i];
        if (!slot.used || slot.port != port || slot.addrLen != addrLen ||
            std::memcmp(slot.addr, addr, addrLen) != 0) {
            continue;
        }
        age = now - slot.ns;
        if (age < 0 || age > kStampAgeNs) {
            slot.used = false;
            continue;
        }
        out->ns = slot.ns;
        std::memcpy(out->mac, slot.mac, 6);
        slot.used = false;
        found     = true;
        break;
    }
    portEXIT_CRITICAL(&s_mux);

    if (found) {
        ++s_stats.hits;
    } else if (s_stats.hits == 0) {
        ESP_LOGW(TAG, "takeRx miss port %u addrLen %u (best age %lld ns; %u recorded)",
                 static_cast<unsigned>(port), static_cast<unsigned>(addrLen),
                 static_cast<long long>(age), static_cast<unsigned>(s_stats.rxStamps));
    }
    return found;
}

Stats stats()
{
    return s_stats;
}

} // namespace ptp
