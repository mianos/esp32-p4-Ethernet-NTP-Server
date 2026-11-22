/*
 * The Ethernet link: EMAC and IP101 PHY over RMII, the netif, and their events.
 *
 * begin() scans MDIO before the netif exists, so a wrong pin map shows (no
 * "MDIO addr" line) before DHCP waits on it, and sets the hostname before
 * esp_eth_start() so the first DHCP DISCOVER carries it. The IPv6 link-local
 * is created on every link-up; esp_netif does not do it. The netif and its
 * glue live for the life of the device.
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>

#include "sdkconfig.h"

#include "esp_err.h"
#include "esp_eth.h"
#include "esp_netif.h"

class EthernetLink {
public:
    static EthernetLink &instance();

    /* The whole bring-up. Call once, after esp_netif_init() and
     * esp_event_loop_create_default(). */
    esp_err_t begin();

    /* Stop and restart the MAC without rebooting. This clears the 1588 block,
     * which is re-enabled from CLOCK_REALTIME and loses its rate steering until
     * the servo pushes it again. Refuses while a restart is in flight; callers
     * rate-limit it. */
    esp_err_t restart(const char *why);

    /* Apply the addressing settings. A static address that does not parse
     * falls back to DHCP. atBoot skips starting the DHCP client, which the
     * netif's connected action does. */
    esp_err_t applySettings(bool atBoot = false);

    /* How many times the link has been restarted this boot. */
    uint32_t restarts() const { return m_restarts; }
    /* Link up with an address; required before an OTA image is confirmed. */
    bool reachable() const;

private:
    EthernetLink() = default;

    /* Owned until the driver is installed, which then owns them. */
    struct MacDeleter {
        void operator()(esp_eth_mac_t *mac) const { mac->del(mac); }
    };
    struct PhyDeleter {
        void operator()(esp_eth_phy_t *phy) const { phy->del(phy); }
    };
    using MacHandle = std::unique_ptr<esp_eth_mac_t, MacDeleter>;
    using PhyHandle = std::unique_ptr<esp_eth_phy_t, PhyDeleter>;

    esp_err_t installDriver();
    esp_err_t attachNetif();
    esp_err_t startStatusTask();

    void scanPhy();

    void logPhyStatus();

    /* nullopt when the SMI read failed. */
    std::optional<uint32_t> readPhyReg(uint32_t reg);

    static void ethEventHandler(void *arg, esp_event_base_t base, int32_t event_id, void *data);
    static void gotIpHandler(void *arg, esp_event_base_t base, int32_t event_id, void *data);
#if CONFIG_LWIP_IPV6
    static void gotIp6Handler(void *arg, esp_event_base_t base, int32_t event_id, void *data);
#endif
    static void statusTaskEntry(void *arg);

    void onEthEvent(int32_t event_id);
    void onGotIp(const ip_event_got_ip_t &event);
    void onAddressUp(const esp_netif_ip_info_t &ip_info);
#if CONFIG_LWIP_IPV6
    void onGotIp6(const ip_event_got_ip6_t &event);
#endif
    void statusLoop();

    esp_eth_handle_t m_eth = nullptr;
    /* Owned by the driver once installed; kept for the 1588 calls. */
    esp_eth_mac_t *m_mac   = nullptr;
    esp_netif_t   *m_netif = nullptr;
    /* Written by the event handlers, read by the status task. */
    std::atomic<bool> m_haveIp{false};
    /* Only ever touched from the event task, so a plain bool is enough. */
    bool m_webStarted = false;
    /* Restart state; the rest is touched only by the holder of m_restarting. */
    std::atomic<bool> m_restarting{false};
    uint32_t          m_restarts      = 0;
    uint32_t          m_autoRestarts  = 0; /* by the link watchdog since the last address */
    int64_t           m_lastRestartUs = 0;

    int64_t m_linkDownSinceUs  = 0;
    bool    m_restartCapLogged = false;
    bool    m_ntpStarted       = false;
    /* The address came from settings, not a lease. */
    bool m_staticAddr = false;
};
