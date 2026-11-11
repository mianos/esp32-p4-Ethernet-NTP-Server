/*
 * Read-only view of the network state for code that is not the Ethernet driver.
 *
 * main.c owns the netif and keeps the PHY link flag up to date from the
 * ETH_EVENT handler; the HTTP server only reads both, so it can report
 * "link_up" and the leased address without a second source of truth.
 */
#pragma once

#include <stdbool.h>

#include "esp_netif.h"

/* The Ethernet netif, or NULL before it is created. */
esp_netif_t *net_state_netif(void);

/* PHY link as last reported by the driver (not the netif admin state). */
bool net_state_link_up(void);
