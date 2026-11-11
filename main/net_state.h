/*
 * Read-only view of the network state for code that is not the Ethernet driver.
 *
 * main.cpp owns the netif and keeps the PHY link flag up to date from the
 * ETH_EVENT handler; the HTTP server only reads both, so it can report
 * "link_up" and the leased address without a second source of truth.
 */
#pragma once

#include "esp_netif.h"

namespace net_state {

/* The Ethernet netif, or nullptr before it is created. */
esp_netif_t *netif();

/* PHY link as last reported by the driver (not the netif admin state). */
bool linkUp();

/* Written only by the Ethernet bring-up and event handlers in main.cpp. */
void setNetif(esp_netif_t *netif);
void setLinkUp(bool up);

}  // namespace net_state

