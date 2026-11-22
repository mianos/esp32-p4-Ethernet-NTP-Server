/* The netif and link state, written by eth_link.cpp and read elsewhere. The
 * IPv6 accessors exist only with CONFIG_LWIP_IPV6. */
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

/* Written only by eth_link.cpp. */
void setNetif(esp_netif_t *netif);
void setLinkUp(bool up);

#if CONFIG_LWIP_IPV6
/* RFC 5952 compressed form, e.g. fe80::200:5eff:fe00:5301. */
std::string ipv6Text(const esp_ip6_addr_t &addr);

/* Preferred addresses only, as text; empty when there are none (a
 * link-local still in DAD does not count). */
std::string ipv6LinkLocal();
std::string ipv6Global();
#endif

}  // namespace net_state
