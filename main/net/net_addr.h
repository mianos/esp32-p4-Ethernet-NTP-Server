/* Strict IPv4 parsing for static addressing, checked when stored and again
 * when programmed. Prefixes 1..30 only. */
#pragma once

#include <cstdint>
#include <string>

#include "esp_netif.h"

namespace net_addr {

/* "a.b.c.d/len" split into a host-order address and a prefix. */
bool parseCidr(const std::string &text, esp_ip4_addr_t &addr, uint8_t &prefix);

/* A bare dotted quad: a gateway or a resolver. */
bool parseQuad(const std::string &text, esp_ip4_addr_t &addr);

/* The netmask a prefix stands for, in the network order esp_netif keeps them. */
esp_ip4_addr_t maskFromPrefix(uint8_t prefix);

/* Dotted-quad text, for logs. */
std::string text(const esp_ip4_addr_t &addr);

}  // namespace net_addr
