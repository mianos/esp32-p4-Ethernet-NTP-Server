/*
 * The sequence that runs after Settings has a new value.
 *
 * Settings owns the values, not the things they drive: the hostname belongs to
 * the netif, the address to eth_link, the time source to the arbiter and the
 * two responders. Both surfaces that change settings -- POST /config and the
 * console -- have to run the same list in the same order, so it lives here
 * rather than being spelled out twice.
 *
 * Save and log stay with the callers: the HTTP handler already reads the blob
 * before and after to decide whether a flash write is warranted, and the
 * console prints its own confirmation.
 */
#pragma once

#include <string>

namespace config_apply {

/* Reapply every setting whose owner caches it. `hostnameBefore` is what the
 * caller saw before the change, so an untouched hostname is not pushed at the
 * netif; `logChanged` says the body named a log key and the sink should be
 * (re)started even if the value ended up identical. */
void afterChange(const std::string &hostnameBefore, bool logChanged);

/* Push a hostname at the netif. The DHCP client embeds it in option 12, but it
 * only builds requests at DISCOVER/REQUEST time, so a change reaches the lease
 * on the next renewal rather than now. */
void hostname(const std::string &name);

}  // namespace config_apply
