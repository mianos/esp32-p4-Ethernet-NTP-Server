/* Reapplies settings after a change, for POST /config and the console alike.
 * The callers save and log. */
#pragma once

#include <string>

namespace config_apply {

/* `hostnameBefore` avoids pushing an unchanged hostname; `logChanged` restarts
 * the log sink even if its value did not change. */
void afterChange(const std::string &hostnameBefore, bool logChanged);

/* The new hostname reaches DHCP at the next renewal. */
void hostname(const std::string &name);

}  // namespace config_apply
