/*
 * Read-only view of the network state for code that is not the Ethernet driver.
 *
 * main.cpp owns the netif and keeps the PHY link flag up to date from the
 * ETH_EVENT handler; the HTTP server only reads both, so it can report
 * "link_up" and the leased address without a second source of truth. The flag
 * is written on the event task and read on the httpd task, hence std::atomic.
 *
 * The IPv6 accessors are compiled only when the project builds with IPv6 (the
 * P4 does; an IPv4-only board can leave CONFIG_LWIP_IPV6 off and nothing here
 * is referenced). They matter because the two families are discovered at
 * different moments: IPv4 at the DHCP bind, IPv6 when lwIP finishes Duplicate
 * Address Detection on the link-local address and again whenever SLAAC adds a
 * global one from a router advertisement.
 */
#pragma once

#include <atomic>

#include "esp_netif.h"
#include "sdkconfig.h"

#if CONFIG_LWIP_IPV6
#include <string>
#endif

namespace net_state {

/* The Ethernet netif, or nullptr before it is created. */
esp_netif_t *netif();

/* PHY link as last reported by the driver (not the netif admin state). */
bool linkUp();

/* Written only by the Ethernet bring-up and event handlers in main.cpp. */
void setNetif(esp_netif_t *netif);
void setLinkUp(bool up);

#if CONFIG_LWIP_IPV6
/* An address in the compressed text form people write by hand
 * (fe80::200:5eff:fe00:5301), not lwIP's eight-group format. */
std::string ipv6Text(const esp_ip6_addr_t &addr);

/* The netif's addresses as text, empty when it has none yet. "Preferred"
 * addresses only: a link-local still in DAD (or one the router deprecated)
 * reads as absent, which is the honest answer for /healthz. */
std::string ipv6LinkLocal();
std::string ipv6Global();
#endif

}  // namespace net_state
