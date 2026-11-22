/* The IP101 is driven by IDF's generic 802.3 PHY driver, which is enough for
 * autonegotiation and link detection. */

#include <inttypes.h>

#include <array>
#include <cstdio>
#include <string>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "net/eth_link.h"
#include "app/cores.h"

#include "soc/emac_reg.h"
#include "soc/soc.h"
#include "net/net_addr.h"
#include "ptp/ptp.h"
#include "net/net_log.h"
#include "net/net_state.h"
#include "ntp/ntp_server.h"
#include "clock/ntp_time.h"
#include "app/settings.h"
#include "http/web_server.h"
#include "app/text.h"

namespace {

constexpr const char *TAG = "eth_demo";

/* Addressing changes come from httpd, the console and the event task.
 * Recursive, because applySettings() calls onAddressUp(). */
SemaphoreHandle_t s_addrLock = nullptr;

struct AddressLock {
    AddressLock() { xSemaphoreTakeRecursive(s_addrLock, portMAX_DELAY); }
    ~AddressLock() { xSemaphoreGiveRecursive(s_addrLock); }
};

/* The stored static address, or false if it does not parse. A bad gateway is
 * dropped, not fatal. */
bool buildIpInfo(const Settings &settings, esp_netif_ip_info_t &info)
{
    esp_ip4_addr_t addr{};
    uint8_t        prefix = 0;
    if (!net_addr::parseCidr(settings.ipAddress(), addr, prefix)) {
        return false;
    }
    info.ip               = addr;
    info.netmask          = net_addr::maskFromPrefix(prefix);
    info.gw               = {};
    const std::string &gw = settings.ipGateway();
    if (!gw.empty() && !net_addr::parseQuad(gw, info.gw)) {
        ESP_LOGW(TAG, "gateway '%s' does not parse -- left unset", gw.c_str());
        info.gw = {};
    }
    return true;
}

/* Set the resolver, or clear it when empty so a lease's does not linger. */
void applyResolver(const std::string &text)
{
    esp_netif_t *netif = net_state::netif();
    if (netif == nullptr) {
        return;
    }
    esp_netif_dns_info_t dns{};
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    if (!text.empty() && !net_addr::parseQuad(text, dns.ip.u_addr.ip4)) {
        ESP_LOGW(TAG, "resolver '%s' does not parse -- left unset", text.c_str());
    }
    esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns);
}

/* ---- Board wiring (Waveshare ESP32-P4-ETH, IP101GRI) --------------------
 * phyReset is active low; phyAddr 1 is set by the board's PHYAD straps. */
struct RmiiPins {
    int mdc;
    int mdio;
    int phyReset;
    int phyAddr;
    int refclk; /* 50 MHz REF_CLK from PHY -> P4 (input) */
    int txEn;
    int txd0;
    int txd1;
    int rxd0;
    int rxd1;
    int crsDv;
};

constexpr RmiiPins kEthPins{
    .mdc      = 31,
    .mdio     = 52,
    .phyReset = 51,
    .phyAddr  = 1,
    .refclk   = 50,
    .txEn     = 49,
    .txd0     = 34,
    .txd1     = 35,
    .rxd0     = 29,
    .rxd1     = 30,
    .crsDv    = 28,
};

/* IEEE 802.3 clause 22 registers. */
constexpr uint32_t kPhyRegBmcr = 0;
constexpr uint32_t kPhyRegBmsr = 1;
constexpr uint32_t kPhyRegId1  = 2;
constexpr uint32_t kPhyRegId2  = 3;

constexpr uint32_t kMdioAddresses = 32;
/* What an empty MDIO address reads. */
constexpr uint32_t kPhyIdAbsent = 0xffff;

constexpr uint32_t kStatusPollMs = 5000;

/* A minute without an address before restarting the MAC, a minute between
 * attempts, and at most five until an address comes back. */
constexpr int64_t  kLinkDownRestartUs = 60000000;
constexpr uint32_t kMaxRestarts       = 5;

constexpr uint32_t kRestartSettleMs = 250;
/* The driver polls MDIO every 2 s and IDF's SMI path has no lock, so the PHY
 * status is read only every sixth tick (30 s). */
constexpr uint32_t    kPhyStatusEvery  = 6;
constexpr uint32_t    kStatusTaskStack = 3072;
constexpr UBaseType_t kStatusTaskPrio  = 3;

std::string macText(const std::array<uint8_t, 6> &mac)
{
    return std::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", mac[0], mac[1], mac[2], mac[3],
                       mac[4], mac[5]);
}

#if CONFIG_LWIP_IPV6

const char *ipv6AddrTypeName(esp_ip6_addr_type_t type)
{
    switch (type) {
    case ESP_IP6_ADDR_IS_GLOBAL:           return "global";
    case ESP_IP6_ADDR_IS_LINK_LOCAL:       return "link-local";
    case ESP_IP6_ADDR_IS_UNIQUE_LOCAL:     return "unique-local";
    case ESP_IP6_ADDR_IS_SITE_LOCAL:       return "site-local";
    case ESP_IP6_ADDR_IS_IPV4_MAPPED_IPV6: return "v4-mapped";
    case ESP_IP6_ADDR_IS_UNKNOWN:          break;
    }
    return "unknown";
}
#endif

void applyPinMap(const RmiiPins &pins, eth_esp32_emac_config_t &config)
{
    config.smi_gpio.mdc_num  = pins.mdc;
    config.smi_gpio.mdio_num = pins.mdio;

    config.emac_dataif_gpio.rmii.tx_en_num  = pins.txEn;
    config.emac_dataif_gpio.rmii.txd0_num   = pins.txd0;
    config.emac_dataif_gpio.rmii.txd1_num   = pins.txd1;
    config.emac_dataif_gpio.rmii.rxd0_num   = pins.rxd0;
    config.emac_dataif_gpio.rmii.rxd1_num   = pins.rxd1;
    config.emac_dataif_gpio.rmii.crs_dv_num = pins.crsDv;
}
/* An OTA image boots pending verification and is confirmed only after it has
 * stayed up and reachable for the hold-off, so one that gets an address and
 * then crashes is still rolled back. */
constexpr int64_t kConfirmHoldoffUs = 120 * 1000000;

esp_timer_handle_t s_confirmTimer = nullptr;

bool imagePendingVerify()
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t   state   = ESP_OTA_IMG_UNDEFINED;

    if (running == nullptr || esp_ota_get_state_partition(running, &state) != ESP_OK) {
        return false;
    }
    return state == ESP_OTA_IMG_PENDING_VERIFY;
}

/* Hold-offs an image may spend unreachable before it is rolled back. */
constexpr int kConfirmTries  = 5;
int           s_confirmTries = 0;

void confirmTimerBody(void *)
{
    if (!imagePendingVerify()) {
        return;
    }

    /* A static address exists without a link, so the link is checked too. */
    if (!EthernetLink::instance().reachable()) {
        if (++s_confirmTries >= kConfirmTries) {
            ESP_LOGE(TAG, "OTA image unreachable for %lld s: rolling back",
                     static_cast<long long>(kConfirmTries * kConfirmHoldoffUs / 1000000));
            esp_ota_mark_app_invalid_rollback_and_reboot();
            return;
        }
        ESP_LOGW(TAG, "OTA image not confirmed: no link or address yet (%d of %d)", s_confirmTries,
                 kConfirmTries);
        esp_timer_start_once(s_confirmTimer, kConfirmHoldoffUs);
        return;
    }

    const esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "OTA image confirmed after %lld s: rollback cancelled",
                 static_cast<long long>((s_confirmTries + 1) * kConfirmHoldoffUs / 1000000));
    } else {
        ESP_LOGW(TAG, "could not confirm the OTA image: %s", esp_err_to_name(err));
    }
}

void confirmRunningImage()
{
    if (!imagePendingVerify()) {
        return; /* first boot after a USB flash: nothing to confirm */
    }
    if (s_confirmTimer != nullptr) {
        return; /* armed already; the lease can be re-acquired */
    }

    esp_timer_create_args_t args = {};
    args.callback                = confirmTimerBody;
    args.name                    = "ota_confirm";
    if (esp_timer_create(&args, &s_confirmTimer) != ESP_OK) {
        /* No timer: confirm now rather than risk rolling back a good image. */
        confirmTimerBody(nullptr);
        return;
    }
    esp_timer_start_once(s_confirmTimer, kConfirmHoldoffUs);
    ESP_LOGI(TAG, "OTA image pending: confirming in %lld s unless it dies first",
             static_cast<long long>(kConfirmHoldoffUs / 1000000));
}

}  // namespace

/* -------------------------------------------------------------------------
 * Bring-up
 * ---------------------------------------------------------------------- */
EthernetLink &EthernetLink::instance()
{
    static EthernetLink link;
    return link;
}

esp_err_t EthernetLink::begin()
{
    s_addrLock = xSemaphoreCreateRecursiveMutex();
    if (s_addrLock == nullptr) {
        ESP_LOGE(TAG, "no memory for the addressing mutex");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = installDriver();
    if (err != ESP_OK) {
        return err;
    }

    scanPhy();
    logPhyStatus();

    err = attachNetif();
    if (err != ESP_OK) {
        return err;
    }

    /* Not fatal: without it NTP uses software stamps. Before the address is
     * applied, since a static address starts PTP, which needs the counter. */
    if (ptp::attach(m_eth, m_mac, m_netif) == ESP_OK) {
        if (ptp::enable() != ESP_OK) {
            ESP_LOGW(TAG, "hardware timestamping unavailable: software stamps only");
        }
    }

    err = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &ethEventHandler, this);
    if (err == ESP_OK) {
        err = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &gotIpHandler, this);
    }
#if CONFIG_LWIP_IPV6
    if (err == ESP_OK) {
        err = esp_event_handler_register(IP_EVENT, IP_EVENT_GOT_IP6, &gotIp6Handler, this);
    }
#endif
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "event handler registration failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Before the MAC starts; under DHCP the netif starts the client at
     * link-up. */
    err = applySettings(true);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_eth_start(m_eth);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_start failed: %s", esp_err_to_name(err));
        return err;
    }
    /* IPv6 needs multicast (ff02::1, solicited-node groups), and esp_eth has
     * no multicast filter API. After start, which reprograms the filter: set
     * promiscuous, and the pass-all-multicast (PM) bit, which has no ioctl. */
#if CONFIG_LWIP_IPV6
    bool            promiscuous    = true; /* the ioctl takes void *, not const */
    const esp_err_t promiscuousErr = esp_eth_ioctl(m_eth, ETH_CMD_S_PROMISCUOUS, &promiscuous);
    if (promiscuousErr != ESP_OK) {
        ESP_LOGW(TAG, "promiscuous mode: %s", esp_err_to_name(promiscuousErr));
    }
    REG_WRITE(EMAC_MACFRAMEFILTER_REG, REG_READ(EMAC_MACFRAMEFILTER_REG) | EMAC_PM_M);
    ESP_LOGI(TAG, "MAC frame filter 0x%08x: multicast passes",
             static_cast<unsigned>(REG_READ(EMAC_MACFRAMEFILTER_REG)));
#endif

    return startStatusTask();
}

esp_err_t EthernetLink::installDriver()
{
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    /* Pin the receive task to the installing core, the work core. */
    mac_config.flags |= ETH_MAC_FLAG_PIN_TO_CORE;
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr         = kEthPins.phyAddr;
    phy_config.reset_gpio_num   = kEthPins.phyReset;

    /* ETH_ESP32_EMAC_DEFAULT_CONFIG() does not compile as C++, so its values
     * are set by hand. */
    eth_esp32_emac_config_t emac_config{};
    applyPinMap(kEthPins, emac_config);

    emac_config.interface = EMAC_DATA_INTERFACE_RMII;
    /* The PHY's 50 MHz REF_CLK output clocks the MAC. */
    emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_config.clock_config.rmii.clock_gpio = kEthPins.refclk;

    emac_config.clock_config_out_in.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_config.clock_config_out_in.rmii.clock_gpio = -1; /* no clock out */
    emac_config.dma_burst_len                       = ETH_DMA_BURST_LEN_32;
    emac_config.intr_priority                       = 0;
    emac_config.mdc_freq_hz                         = 0; /* driver default */

    MacHandle mac(esp_eth_mac_new_esp32(&emac_config, &mac_config));
    if (!mac) {
        ESP_LOGE(TAG, "failed to create EMAC instance");
        return ESP_FAIL;
    }

    PhyHandle phy(esp_eth_phy_new_generic(&phy_config));
    if (!phy) {
        ESP_LOGE(TAG, "failed to create PHY instance");
        return ESP_FAIL;
    }

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac.get(), phy.get());
    const esp_err_t  err        = esp_eth_driver_install(&eth_config, &m_eth);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    /* For the 1588 calls; the driver keeps it alive. */
    m_mac = mac.get();

    mac.release();
    phy.release();
    return ESP_OK;
}
esp_err_t EthernetLink::attachNetif()
{
    const esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    m_netif                               = esp_netif_new(&netif_config);
    if (m_netif == nullptr) {
        ESP_LOGE(TAG, "esp_netif_new failed");
        return ESP_FAIL;
    }
    net_state::setNetif(m_netif);

    /* Before esp_eth_start(), for DHCP option 12. Not fatal: a bad stored name
     * must not stop the boot. */
    const std::string &hostname = Settings::instance().hostname();
    const esp_err_t    err      = esp_netif_set_hostname(m_netif, hostname.c_str());
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_netif_set_hostname('%s'): %s -- keeping lwIP's default",
                 hostname.c_str(), esp_err_to_name(err));
    }

    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(m_eth);
    if (glue == nullptr) {
        ESP_LOGE(TAG, "esp_eth_new_netif_glue failed");
        return ESP_FAIL;
    }
    return esp_netif_attach(m_netif, glue);
}

esp_err_t EthernetLink::applySettings(bool atBoot)
{
    if (m_netif == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    const Settings     &settings   = Settings::instance();
    const bool          wantStatic = settings.ipMode() == "static";
    esp_netif_ip_info_t info{};
    const bool          usable = wantStatic && buildIpInfo(settings, info);

    AddressLock lock;

    if (usable) {
        /* Stop DHCP so a pending lease cannot overwrite the address. */
        esp_netif_dhcpc_stop(m_netif);
        esp_err_t err = esp_netif_set_ip_info(m_netif, &info);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_netif_set_ip_info: %s -- falling back to DHCP",
                     esp_err_to_name(err));
            esp_netif_dhcpc_start(m_netif);
            m_staticAddr = false;
            return err;
        }
        esp_netif_set_default_netif(m_netif);
        applyResolver(settings.ipDns());
        m_staticAddr = true;
        /* set_ip_info() does not reliably post GOT_IP. */
        onAddressUp(info);
        return ESP_OK;
    }

    if (wantStatic) {
        ESP_LOGW(TAG, "ipMode=static but '%s' does not parse -- staying on DHCP",
                 settings.ipAddress().c_str());
    }
    m_staticAddr = false;
    if (atBoot) {
        return ESP_OK;
    }
    /* ALREADY_STARTED is normal here. */
    esp_netif_dhcpc_start(m_netif);
    return ESP_OK;
}

esp_err_t EthernetLink::restart(const char *why)
{
    bool expected = false;
    if (!m_restarting.compare_exchange_strong(expected, true)) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG, "restarting the MAC (%s) -- %" PRIu32 " this boot, no reboot", why, m_restarts);
    esp_err_t err = esp_eth_stop(m_eth);
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(kRestartSettleMs));
        err = esp_eth_start(m_eth);
    }
    if (err == ESP_OK) {
        /* stop/start clears the 1588 block with the rest of the MAC. */
        ptp::enable(true);
        ++m_restarts;
        /* Log what autonegotiation settled on. */
        logPhyStatus();
    } else {
        ESP_LOGE(TAG, "MAC restart failed: %s", esp_err_to_name(err));
    }
    m_lastRestartUs = esp_timer_get_time();
    m_restarting.store(false);
    return err;
}

esp_err_t EthernetLink::startStatusTask()
{
    if (xTaskCreatePinnedToCore(&statusTaskEntry, "eth_status", kStatusTaskStack, this,
                                kStatusTaskPrio, nullptr, cores::kWork) != pdPASS) {
        ESP_LOGW(TAG, "could not start the link status task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * PHY diagnostics
 * ---------------------------------------------------------------------- */
std::optional<uint32_t> EthernetLink::readPhyReg(uint32_t reg)
{
    uint32_t                  value = 0;
    esp_eth_phy_reg_rw_data_t request{};
    request.reg_addr    = reg;
    request.reg_value_p = &value;
    if (esp_eth_ioctl(m_eth, ETH_CMD_READ_PHY_REG, &request) != ESP_OK) {
        return std::nullopt;
    }
    return value;
}

void EthernetLink::scanPhy()
{
    for (uint32_t addr = 0; addr < kMdioAddresses; addr++) {
        esp_eth_ioctl(m_eth, ETH_CMD_S_PHY_ADDR, &addr);
        const std::optional<uint32_t> id1 = readPhyReg(kPhyRegId1);
        const std::optional<uint32_t> id2 = readPhyReg(kPhyRegId2);
        if (!id1 || !id2 || *id1 == 0 || *id1 == kPhyIdAbsent) {
            continue;
        }
        ESP_LOGI(TAG,
                 "MDIO addr %" PRIu32 ": PHY ID 0x%04" PRIx32 "%04" PRIx32 " (IP101 = 0x0243xxxx)",
                 addr, *id1, *id2);
    }

    uint32_t phy_addr = static_cast<uint32_t>(kEthPins.phyAddr);
    esp_eth_ioctl(m_eth, ETH_CMD_S_PHY_ADDR, &phy_addr);
}

void EthernetLink::logPhyStatus()
{
    const std::optional<uint32_t> bmsr = readPhyReg(kPhyRegBmsr);
    const std::optional<uint32_t> bmcr = readPhyReg(kPhyRegBmcr);
    if (!bmsr || !bmcr) {
        ESP_LOGW(TAG, "phy: MDIO read failed (check MDC/MDIO wiring; the driver "
                      "reads MDIO too, so a busy bus looks the same)");
        return;
    }
    ESP_LOGI(TAG, "phy: bmsr=0x%04" PRIx32 " link=%d autoneg_done=%d bmcr=0x%04" PRIx32, *bmsr,
             static_cast<int>((*bmsr >> 2) & 1u), static_cast<int>((*bmsr >> 5) & 1u), *bmcr);
}

/* -------------------------------------------------------------------------
 * Event handlers: trampolines to the instance passed as `arg`.
 * ---------------------------------------------------------------------- */
void EthernetLink::ethEventHandler(void *arg, esp_event_base_t, int32_t event_id, void *)
{
    static_cast<EthernetLink *>(arg)->onEthEvent(event_id);
}

void EthernetLink::onEthEvent(int32_t event_id)
{
    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED: {
        std::array<uint8_t, 6> mac{};
        esp_eth_ioctl(m_eth, ETH_CMD_G_MAC_ADDR, mac.data());
        net_state::setLinkUp(true);
        ESP_LOGI(TAG, "link up   (MAC %s)", macText(mac).c_str());
        ESP_LOGI(TAG, "requesting DHCP lease ...");
#if CONFIG_LWIP_IPV6
        /* esp_netif does not create the link-local itself, and SLAAC needs it
         * to send the Router Solicitation. Every link-up, so a cable bounce
         * re-arms it. */
        if (m_netif != nullptr) {
            const esp_err_t err = esp_netif_create_ip6_linklocal(m_netif);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "IPv6 link-local: %s", esp_err_to_name(err));
            }
        }
#endif
        break;
    }
    case ETHERNET_EVENT_DISCONNECTED:
        m_haveIp.store(false, std::memory_order_relaxed);
        net_state::setLinkUp(false);
        ESP_LOGW(TAG, "link down");
        break;
    case ETHERNET_EVENT_START: ESP_LOGI(TAG, "Ethernet started"); break;
    case ETHERNET_EVENT_STOP:  ESP_LOGI(TAG, "Ethernet stopped"); break;
    default:                   break;
    }
}

void EthernetLink::gotIpHandler(void *arg, esp_event_base_t, int32_t, void *event_data)
{
    auto *event = static_cast<ip_event_got_ip_t *>(event_data);
    static_cast<EthernetLink *>(arg)->onGotIp(*event);
}

void EthernetLink::onGotIp(const ip_event_got_ip_t &event)
{
    onAddressUp(event.ip_info);
}

/* A lease or a static address is up: start what needs an address. */
void EthernetLink::onAddressUp(const esp_netif_ip_info_t &ip_info)
{
    m_haveIp.store(true, std::memory_order_relaxed);
    ESP_LOGI(TAG, "%s: ip " IPSTR "  mask " IPSTR "  gw " IPSTR,
             m_staticAddr ? "static address" : "DHCP lease", IP2STR(&ip_info.ip),
             IP2STR(&ip_info.netmask), IP2STR(&ip_info.gw));
    ESP_LOGI(TAG, "ready: ping the IP above from your LAN -- ICMP echo is answered by lwIP");

    /* A lease may carry an option 42 NTP server. */
    NtpTime::instance().onGotIp();

    /* HTTP on the first address; it listens on all addresses after that. */
    if (!m_webStarted) {
        const esp_err_t err = NtpWebServer::instance().begin();
        if (err == ESP_OK) {
            m_webStarted = true;
            net_log::start();
            ESP_LOGI(TAG, "http: http://" IPSTR "/healthz  http://" IPSTR "/config",
                     IP2STR(&ip_info.ip), IP2STR(&ip_info.ip));
            confirmRunningImage();
        } else {
            ESP_LOGE(TAG, "http server failed to start: %s", esp_err_to_name(err));
        }
    }

    /* PTP needs no address, but is started here with the other services. */
    const std::string &ptpMode = Settings::instance().ptpMode();
    if (ptp::mode() == ptp::Mode::Off && (ptpMode == "master" || ptpMode == "slave")) {
        (void)ptp::start(ptpMode == "master" ? ptp::Mode::Master : ptp::Mode::Slave,
                         Settings::instance().ptpDomain(), Settings::instance().ptpPriority1());
    }

    if (!m_ntpStarted) {
        if (NtpServer::instance().begin() == ESP_OK) {
            m_ntpStarted = true;
        } else {
            ESP_LOGE(TAG, "ntp responder failed to start on udp/123");
        }
    }
}

#if CONFIG_LWIP_IPV6
/* GOT_IP6 is posted per address: the link-local after DAD, then each SLAAC
 * address. Each is logged with its URL. */
void EthernetLink::gotIp6Handler(void *arg, esp_event_base_t, int32_t, void *event_data)
{
    auto *event = static_cast<ip_event_got_ip6_t *>(event_data);
    static_cast<EthernetLink *>(arg)->onGotIp6(*event);
}

void EthernetLink::onGotIp6(const ip_event_got_ip6_t &event)
{
    const std::string addr = net_state::ipv6Text(event.ip6_info.ip);
    ESP_LOGI(TAG, "IPv6 %s: %s", ipv6AddrTypeName(esp_netif_ip6_get_addr_type(&event.ip6_info.ip)),
             addr.c_str());
    ESP_LOGI(TAG, "http over IPv6: http://[%s]/healthz", addr.c_str());
}
#endif

/* -------------------------------------------------------------------------
 * Status task: logs while there is no address, and restarts the MAC if that
 * lasts.
 * ---------------------------------------------------------------------- */
bool EthernetLink::reachable() const
{
    return m_haveIp.load(std::memory_order_relaxed) && net_state::linkUp();
}

void EthernetLink::statusTaskEntry(void *arg)
{
    static_cast<EthernetLink *>(arg)->statusLoop();
}

void EthernetLink::statusLoop()
{
    uint32_t tick = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(kStatusPollMs));
        if (m_haveIp.load(std::memory_order_relaxed)) {
            m_linkDownSinceUs = 0;
            /* An address resets the restart budget. */
            m_autoRestarts     = 0;
            m_restartCapLogged = false;
            continue;
        }

        const int64_t nowUs = esp_timer_get_time();
        if (m_linkDownSinceUs == 0) {
            m_linkDownSinceUs = nowUs;
        }
        ESP_LOGI(TAG, "waiting for link + address (is the RJ45 plugged in?) -- %lld s",
                 static_cast<long long>((nowUs - m_linkDownSinceUs) / 1000000));
        if (tick++ % kPhyStatusEvery == 0) {
            logPhyStatus();
        }

        /* Without an address for kLinkDownRestartUs, restart the MAC. */
        if (nowUs - m_linkDownSinceUs < kLinkDownRestartUs || m_restarting) {
            continue;
        }
        if (m_autoRestarts >= kMaxRestarts) {
            if (!m_restartCapLogged) {
                m_restartCapLogged = true;
                ESP_LOGE(TAG, "link down after %" PRIu32 " restarts -- needs a power cycle",
                         m_restarts);
            }
            continue;
        }
        ++m_autoRestarts;
        restart("link down");
        m_linkDownSinceUs = esp_timer_get_time();
    }
}
