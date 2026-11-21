/*
 * IPv4 literals, parsed once and validated hard.
 *
 * A static address is the one setting that can make the box unreachable, so it
 * is checked where it is written (settings.cpp, from POST /config or the
 * console) and again where it is programmed (eth_link.cpp). The prefix is the
 * only way to state a netmask, and 1..30 is the only range that leaves both a
 * host and a broadcast.
 */
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
