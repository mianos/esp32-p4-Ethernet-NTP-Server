/*
 * The Ethernet link: EMAC + IP101 PHY over RMII, the netif on top of both, and
 * the events they raise.
 *
 * The RJ45 is the only field interface this board has, so the bring-up and its
 * diagnostics live here and main.cpp keeps nothing but boot order.
 *
 * Two ordering details decide the shape of begin():
 *
 *  - the PHY diagnostic (scan the 32 MDIO addresses, then read BMSR/BMCR) runs
 *    before the netif exists, so a wrong pin map shows up before DHCP is
 *    waiting on it. Every address reading 0xffff/0 and no "MDIO addr" line at
 *    all is the answer to "are my pins right?";
 *  - the hostname goes in before esp_eth_start(), because DHCP option 12 is
 *    built when the client's first DISCOVER leaves, not after the lease.
 *
 * IPv6 has to be asked for: esp_netif does not create the fe80:: link-local on
 * its own, and without a source address lwIP has nothing to send the Router
 * Solicitation from -- so no advertisement comes back and there is no global
 * address either. onEthEvent() calls esp_netif_create_ip6_linklocal() on every
 * link-up, so a cable bounce re-arms it. Those parts are compiled only when
 * CONFIG_LWIP_IPV6 is on; the same source serves an IPv4-only board otherwise.
 *
 * Ownership is the instance's: the MAC and PHY are unique_ptrs with del()
 * deleters until the driver takes them over (after which it deletes them), the
 * netif and the glue it was given are never freed -- they live as long as the
 * device -- and begin() returns the error rather than aborting halfway through.
 * app_main() is what decides an error is fatal.
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

    /* Driver, PHY diagnostic, netif, handlers, status task: the whole bring-up.
     * Call once, after esp_netif_init() and esp_event_loop_create_default(). */
    esp_err_t begin();

    /* Stop and restart the MAC, without rebooting and without touching the
     * clock. The anchor is esp_timer, which the MAC has nothing to do with, so
     * unlike a PTP grandmaster there is no PHC to preserve here -- the cost of
     * a restart is the ~2 s of link plus a re-lease, against a reboot's 20 s of
     * stratum 16 and every counter on the status page going back to zero.
     *
     * Refuses while a restart is already in flight. Rate-limited by the caller
     * where it is a watchdog, unlimited where it is a human on the test hook. */
    esp_err_t restart(const char *why);

    /* Program the netif from the addressing settings and bring the address up.
     * Called at boot and by every surface that changes them (POST /config, the
     * console). Falls back to DHCP rather than failing hard when a stored static
     * value does not parse; atBoot suppresses the client restart, which the
     * netif's own connected action would duplicate. */
    esp_err_t applySettings(bool atBoot = false);

    /* How many times the link has been restarted this boot. */
    uint32_t restarts() const { return m_restarts; }
    /* An address on a link that is up: what an OTA'd image has to show before
     * it is kept. */
    bool reachable() const;

private:
    EthernetLink() = default;

    /* esp_eth hands out C objects with their own del(), so a unique_ptr is the
     * whole reason installDriver() has no cleanup branches. */
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

    /* Scan the 32 MDIO addresses for a PHY and print what is there. */
    void scanPhy();

    /* One line of PHY status: BMSR gives link + autoneg-done, BMCR the rest. */
    void logPhyStatus();

    /* std::nullopt when the SMI read itself failed. */
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
    /* Restart bookkeeping: the in-flight guard is checked from a task, so it
     * is atomic; the rest is only touched by whoever holds the guard. */
    std::atomic<bool> m_restarting{false};
    uint32_t          m_restarts      = 0;
    uint32_t          m_autoRestarts  = 0; /* by the link watchdog since the last address */
    int64_t           m_lastRestartUs = 0;
    /* When the current run of "no lease" began, and whether the attempt cap has
     * already been reported (it would otherwise repeat every poll). */
    int64_t m_linkDownSinceUs  = 0;
    bool    m_restartCapLogged = false;
    bool    m_ntpStarted       = false;
    /* Set while the active address came from the settings rather than a lease:
     * it picks the log wording and keeps the two paths honest. */
    bool m_staticAddr = false;
};
