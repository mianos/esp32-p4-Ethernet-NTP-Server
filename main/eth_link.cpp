/*
 * Ethernet link: EMAC (RMII) + generic IEEE 802.3 PHY driver, the netif on top
 * of both, and the events they raise. See eth_link.h for why the order is what
 * it is; this file is the mechanics.
 *
 * The generic PHY driver speaks the standard MII registers, which is all the
 * IP101 needs for autonegotiation and link-up detection.
 */

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

#include "eth_link.h"

#include "soc/emac_reg.h"
#include "soc/soc.h"
#include "net_addr.h"
#include "net_log.h"
#include "net_state.h"
#include "ntp_server.h"
#include "ntp_time.h"
#include "settings.h"
#include "web_server.h"
#include "text.h"

namespace {

/* The device logs under one tag whichever file emits the line. */
constexpr const char *TAG = "eth_demo";

/* Addressing is changed from the httpd task and from the console task, while
 * the driver's event task may be walking the same netif. Recursive, because
 * applySettings() calls onAddressUp() and the CONNECTED path can nest there. */
SemaphoreHandle_t s_addrLock = nullptr;

struct AddressLock {
    AddressLock() { xSemaphoreTakeRecursive(s_addrLock, portMAX_DELAY); }
    ~AddressLock() { xSemaphoreGiveRecursive(s_addrLock); }
};

/* The stored static address as esp_netif wants it, or false if it does not
 * parse. The gateway is optional and a bad one is dropped rather than fatal:
 * a box with an address and no default route is still reachable on the wire. */
bool buildIpInfo(const Settings &settings, esp_netif_ip_info_t &info)
{
    esp_ip4_addr_t addr{};
    uint8_t        prefix = 0;
    if (!net_addr::parseCidr(settings.ipAddress(), addr, prefix)) {
        return false;
    }
    info.ip      = addr;
    info.netmask = net_addr::maskFromPrefix(prefix);
    info.gw      = {};
    const std::string &gw = settings.ipGateway();
    if (!gw.empty() && !net_addr::parseQuad(gw, info.gw)) {
        ESP_LOGW(TAG, "gateway '%s' does not parse -- left unset", gw.c_str());
        info.gw = {};
    }
    return true;
}

/* Resolver: set, or cleared when the setting is empty (a DHCP lease would
 * otherwise leave its own behind). */
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
 * Reference design values. MDC/MDIO are the SMI (management) bus, the rest
 * are the RMII data plane. phyReset is active low; phyAddr 1 is the IP101
 * default set by the PHYAD strap resistors on the board.
 */
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

/* IEEE 802.3 clause 22 register numbers, as the PHY diagnostic reads them. */
constexpr uint32_t kPhyRegBmcr = 0;
constexpr uint32_t kPhyRegBmsr = 1;
constexpr uint32_t kPhyRegId1  = 2;
constexpr uint32_t kPhyRegId2  = 3;

constexpr uint32_t kMdioAddresses = 32;
/* What a float-bus address reads back: no PHY there. */
constexpr uint32_t kPhyIdAbsent = 0xffff;

constexpr uint32_t    kStatusPollMs    = 5000;

/* A minute down before the MAC is assumed wedged rather than the cable being
 * out: a switch reboot, a patch lead being moved and a DHCP server restart all
 * finish inside that. The down-window restarts after every attempt, so the
 * attempts space themselves by a minute too, and five of them is where a reboot
 * becomes the honest answer. */
constexpr int64_t     kLinkDownRestartUs = 60000000;
constexpr uint32_t    kMaxRestarts       = 5;
/* Millisecond-scale pause after stopping, so the PHY sees a real break in the
 * link rather than a single sample of it. */
constexpr uint32_t    kRestartSettleMs   = 250;
/* The driver polls the link over MDIO by itself every 2 s, and IDF's SMI path
 * has no mutex: two readers can return each other's data, or ESP_ERR_INVALID_STATE.
 * So the registers are read once every six ticks (30 s) while the "waiting"
 * line keeps its 5 s rhythm. */
constexpr uint32_t    kPhyStatusEvery  = 6;
constexpr uint32_t    kStatusTaskStack = 3072;
constexpr UBaseType_t kStatusTaskPrio  = 3;

/* 02:00:5e:00:53:01 -- the form the rest of the documentation uses. */
std::string macText(const std::array<uint8_t, 6> &mac)
{
    return std::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", mac[0], mac[1], mac[2], mac[3],
                       mac[4], mac[5]);
}

#if CONFIG_LWIP_IPV6
/* The word the log and /healthz use for an address class. */
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

/* Copy this board's pin map into the EMAC config. Only the pads live here:
 * interface mode, clocking and DMA stay in installDriver() with the driver
 * settings they belong to. */
void applyPinMap(const RmiiPins &pins, eth_esp32_emac_config_t &config)
{
    config.smi_gpio.mdc_num  = pins.mdc;
    config.smi_gpio.mdio_num = pins.mdio;

    /* P4 routes the RMII data signals through the GPIO matrix. */
    config.emac_dataif_gpio.rmii.tx_en_num  = pins.txEn;
    config.emac_dataif_gpio.rmii.txd0_num   = pins.txd0;
    config.emac_dataif_gpio.rmii.txd1_num   = pins.txd1;
    config.emac_dataif_gpio.rmii.rxd0_num   = pins.rxd0;
    config.emac_dataif_gpio.rmii.rxd1_num   = pins.rxd1;
    config.emac_dataif_gpio.rmii.crs_dv_num = pins.crsDv;
}
/* An image that arrived over POST /firmware boots in ESP_OTA_IMG_PENDING_VERIFY:
 * the bootloader goes back to the previous slot unless the app confirms this
 * one.
 *
 * Confirming on the lease alone was not enough. A lease arrives about seven
 * seconds in, so an image that gets an address and then panics -- which is
 * exactly what a sky-view sized Status did, every ten seconds for nine hours --
 * confirms itself and the rollback never fires. Reachable for a moment is not
 * the same as working.
 *
 * So confirmation waits out a hold-off first. If the image dies inside it, it
 * stays unverified and the bootloader rolls back on the next reset, which is the
 * only recovery path on a box whose interface is the RJ45. The timer is a
 * one-shot and the state is re-read when it fires, so an image that a USB flash
 * put here (nothing pending) confirms nothing.
 */
constexpr int64_t kConfirmHoldoffUs = 120 * 1000000;

esp_timer_handle_t s_confirmTimer = nullptr;

/* True while the running image is still on probation. Read at both sites rather
 * than once: the timer fires 120 s after a lease, and the image could have been
 * confirmed by something else (or rolled back) in between. */
bool imagePendingVerify()
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t   state   = ESP_OTA_IMG_UNDEFINED;

    if (running == nullptr || esp_ota_get_state_partition(running, &state) != ESP_OK) {
        return false;
    }
    return state == ESP_OTA_IMG_PENDING_VERIFY;
}

void confirmTimerBody(void *)
{
    if (!imagePendingVerify()) {
        return;
    }

    const esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "OTA image confirmed after %lld s: rollback cancelled",
                 static_cast<long long>(kConfirmHoldoffUs / 1000000));
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
    args.callback = confirmTimerBody;
    args.name     = "ota_confirm";
    if (esp_timer_create(&args, &s_confirmTimer) != ESP_OK) {
        /* No timer: fall back to confirming now rather than risk a good image
         * being rolled back for want of a timer. */
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

    /* Which PHY answers on the SMI bus, and what does it see? Both before the
     * netif exists, so a wiring fault shows up before DHCP is waiting on it. */
    scanPhy();
    logPhyStatus();

    err = attachNetif();
    if (err != ESP_OK) {
        return err;
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

    /* Addressing before the MAC: with the DHCP client stopped and a valid
     * static address in place, the netif's own connected action posts GOT_IP
     * for it, so the rest of the bring-up needs no static case at all. Under
     * DHCP this is a no-op and the default action starts the client at link-up,
     * which is why the boot-time call passes atBoot. */
    err = applySettings(true);
    if (err != ESP_OK) {
        return err;
    }

    /* Multicast has to be let through by hand. esp_eth exposes no multicast
     * hash, only promiscuous mode, and with the MAC filtering to its own address
     * the board never hears a router advertisement or a neighbour solicitation:
     * it has a link-local, answers unicast, and is otherwise invisible to IPv6.
     * SLAAC and NDP both need ff02::1 and the solicited-node groups, so the
     * choice is promiscuous or no IPv6 at all. The port is the box's own, so the
     * extra frames are the segment's broadcast traffic and nothing else. */
    err = esp_eth_start(m_eth);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_start failed: %s", esp_err_to_name(err));
        return err;
    }
    /* After start, not before: the MAC is programmed during start and that
     * clears the filter bits, so a promiscuous set beforehand is thrown away. */
#if CONFIG_LWIP_IPV6
    /* Two filter bits, and only one of them has an ioctl. PRMODE passes frames
     * whatever their address; the separate PM bit is the one that passes
     * *multicast*, and promiscuous does not imply it. Without PM the MAC drops
     * ff02::1 and the solicited-node groups, so the board has a link-local,
     * answers a unicast ping, and never hears an advertisement or a neighbour
     * solicitation -- SLAAC cannot work and neither can anything else. The HAL
     * has a wrapper for PM (emac_ll_pass_all_multicast_enable) but no ioctl
     * reaches it, so the register is written here. */
    bool promiscuous = true; /* the ioctl takes void *, not const */
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
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr       = kEthPins.phyAddr;
    phy_config.reset_gpio_num = kEthPins.phyReset;

    /* ETH_ESP32_EMAC_DEFAULT_CONFIG() is written with C99 designated
     * initialisers in an order C++ rejects, so its defaults are set by hand
     * here -- same values, plus this board's pin map. */
    eth_esp32_emac_config_t emac_config{};
    applyPinMap(kEthPins, emac_config);

    emac_config.interface = EMAC_DATA_INTERFACE_RMII;
    /* The PHY's 50 MHz REF_CLK output clocks the MAC. */
    emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_config.clock_config.rmii.clock_gpio = kEthPins.refclk;

    emac_config.clock_config_out_in.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_config.clock_config_out_in.rmii.clock_gpio = -1; /* no clock out */
    emac_config.dma_burst_len = ETH_DMA_BURST_LEN_32;
    emac_config.intr_priority = 0;
    emac_config.mdc_freq_hz   = 0; /* driver default */

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
    const esp_err_t err = esp_eth_driver_install(&eth_config, &m_eth);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Installed: the driver owns both instances from here. */
    mac.release();
    phy.release();
    return ESP_OK;
}
esp_err_t EthernetLink::attachNetif()
{
    /* Default Ethernet netif: DHCP client on, ICMP echo on (lwIP default). */
    const esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    m_netif = esp_netif_new(&netif_config);
    if (m_netif == nullptr) {
        ESP_LOGE(TAG, "esp_netif_new failed");
        return ESP_FAIL;
    }
    net_state::setNetif(m_netif);

    /* DHCP option 12 is built when the client's first DISCOVER goes out, which
     * happens once the netif comes up -- so the hostname has to be in place
     * before esp_eth_start(), not after the first lease.
     *
     * Deliberately not fatal. app_main() turns a begin() error into an abort,
     * and a device that cannot boot has no way back in -- so a stored hostname
     * must not be able to stop it. Settings caps the name at esp_netif's own
     * limit (32), which makes this a backstop rather than the fix. */
    const std::string &hostname = Settings::instance().hostname();
    const esp_err_t    err      = esp_netif_set_hostname(m_netif, hostname.c_str());
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_netif_set_hostname('%s'): %s -- keeping lwIP's default",
                 hostname.c_str(), esp_err_to_name(err));
    }

    /* The glue is handed to the netif, which owns it from here. */
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

    const Settings     &settings = Settings::instance();
    const bool          wantStatic = settings.ipMode() == "static";
    esp_netif_ip_info_t info{};
    const bool          usable = wantStatic && buildIpInfo(settings, info);

    AddressLock lock;

    if (usable) {
        /* Tolerated: the client is normally already stopped by the previous
         * static apply, and this is what actually keeps a stored DHCP request
         * from overwriting the address a second later. */
        esp_netif_dhcpc_stop(m_netif);
        esp_err_t err = esp_netif_set_ip_info(m_netif, &info);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_netif_set_ip_info: %s -- falling back to DHCP", esp_err_to_name(err));
            esp_netif_dhcpc_start(m_netif);
            m_staticAddr = false;
            return err;
        }
        esp_netif_set_default_netif(m_netif);
        applyResolver(settings.ipDns());
        m_staticAddr = true;
        /* set_ip_info() does not reliably post GOT_IP the way a lease does, so
         * the address-up path is entered directly -- and it is also where the
         * address gets logged, so there is only one line per change. */
        onAddressUp(info);
        return ESP_OK;
    }

    if (wantStatic) {
        ESP_LOGW(TAG, "ipMode=static but '%s' does not parse -- staying on DHCP",
                 settings.ipAddress().c_str());
    }
    m_staticAddr = false;
    if (atBoot) {
        /* The connected action starts the client; starting it here as well
         * would only race that. */
        return ESP_OK;
    }
    /* ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED is the normal case for a request
     * that changed something else. */
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
        ++m_restarts;
        /* The PHY comes back with whatever autoneg settles on; say so, because
         * "restarted and still 100 Mb/s" and "restarted into a half-duplex
         * 10 Mb/s" are different outcomes for the servo's uplink. */
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
    if (xTaskCreate(&statusTaskEntry, "eth_status", kStatusTaskStack, this, kStatusTaskPrio,
                    nullptr) != pdPASS) {
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
    uint32_t value = 0;
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
        ESP_LOGI(TAG, "MDIO addr %" PRIu32 ": PHY ID 0x%04" PRIx32 "%04" PRIx32
                 " (IP101 = 0x0243xxxx)", addr, *id1, *id2);
    }

    /* Leave the driver pointed at the PHY it was configured with. */
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
    ESP_LOGI(TAG, "phy: bmsr=0x%04" PRIx32 " link=%d autoneg_done=%d bmcr=0x%04" PRIx32,
             *bmsr, static_cast<int>((*bmsr >> 2) & 1u),
             static_cast<int>((*bmsr >> 5) & 1u), *bmcr);
}

/* -------------------------------------------------------------------------
 * Event handlers. esp_event passes the instance as `arg`, so these are thin
 * trampolines and the real work is a member function with typed arguments.
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
        /* IPv6 has to be asked for: esp_netif does not create the fe80::
         * link-local on its own, and without one lwIP has no source address for
         * the Router Solicitation that would bring a SLAAC address back. Doing
         * it on every link-up, as the IDF examples do, means a cable bounce
         * re-arms it rather than leaving the board IPv4-only until reboot. */
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
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet stopped");
        break;
    default:
        break;
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

/* The address is up, however it got here: a lease, or a static one the console
 * or POST /config programmed. Everything below hangs off having an address, so
 * both enter it through here rather than through two copies. */
void EthernetLink::onAddressUp(const esp_netif_ip_info_t &ip_info)
{
    m_haveIp.store(true, std::memory_order_relaxed);
    ESP_LOGI(TAG, "%s: ip " IPSTR "  mask " IPSTR "  gw " IPSTR,
             m_staticAddr ? "static address" : "DHCP lease",
             IP2STR(&ip_info.ip), IP2STR(&ip_info.netmask), IP2STR(&ip_info.gw));
    ESP_LOGI(TAG, "ready: ping the IP above from your LAN -- ICMP echo is answered by lwIP");

    /* The lease is where the option-42 NTP server (if any) becomes known, so
     * the clock can poll it now rather than at the next SNTP interval. */
    NtpTime::instance().onGotIp();

    /* The HTTP surface is only reachable once there is an address, so start it
     * on the first lease. Later rebinds leave it listening on all addresses. */
    if (!m_webStarted) {
        const esp_err_t err = NtpWebServer::instance().begin();
        if (err == ESP_OK) {
            m_webStarted = true;
            net_log::start();
            ESP_LOGI(TAG, "http: http://" IPSTR "/healthz  http://" IPSTR "/config",
                     IP2STR(&ip_info.ip), IP2STR(&ip_info.ip));
            /* Reachable is the condition for keeping an OTA'd image; the slot
             * only needs confirming once, on its first boot. */
            confirmRunningImage();
        } else {
            ESP_LOGE(TAG, "http server failed to start: %s", esp_err_to_name(err));
        }
    }

    /* The responder binds :123 -- also an address-dependent thing. It serves
     * whatever the clock is: stratum 3 from the upstream, or 16 with the leap
     * indicator at 3 while the clock is not locked (i.e. "ignore me"). */
    if (!m_ntpStarted) {
        if (NtpServer::instance().begin() == ESP_OK) {
            m_ntpStarted = true;
        } else {
            ESP_LOGE(TAG, "ntp responder failed to start on udp/123");
        }
    }
}

#if CONFIG_LWIP_IPV6
/* IPv6 arrives on its own event, per address: esp_netif posts GOT_IP6 once DAD
 * has cleared the fe80:: the link-up handler asked for, and again every time
 * SLAAC adds or re-adds a global address from a router advertisement (prefix
 * changes, router restart). Logging each one with the URL it makes reachable is
 * the only way to learn the address without a packet capture -- a link-local
 * one only works from the same segment, hence "[addr]" rather than a host name.
 */
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
 * Status task: says what the demo is waiting for, then goes quiet.
 * ---------------------------------------------------------------------- */
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
            m_linkDownSinceUs = 0; /* the lease is in: quiet, but still watching */
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

        /* Nothing else in this firmware ever touches the MAC again, so a PHY
         * that stopped answering stays stopped until someone reboots the board
         * by hand -- and the box is usually the far end of the link that would
         * have told you about it. */
        if (nowUs - m_linkDownSinceUs < kLinkDownRestartUs || m_restarting) {
            continue;
        }
        if (m_restarts >= kMaxRestarts) {
            if (!m_restartCapLogged) {
                m_restartCapLogged = true;
                ESP_LOGE(TAG, "link down after %" PRIu32 " restarts -- needs a power cycle",
                         m_restarts);
            }
            continue;
        }
        restart("link down");
        m_linkDownSinceUs = esp_timer_get_time();
    }
}

