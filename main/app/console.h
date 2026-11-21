/*
 * The serial console.
 *
 * It exists for the things that have to work when the network does not: set a
 * static address, go back to DHCP, reset the config, or just read what the box
 * thinks it is doing. USB is the only door -- the box has no other reachable
 * port -- so this is the recovery path that survives a wrong address, and the
 * reason eth_link refuses to accept a static address it cannot parse.
 *
 * Everything goes through Settings::applyJson(), the same validation POST
 * /config uses, so the console cannot invent a state the HTTP surface rejects.
 */
#pragma once

#include "esp_err.h"

namespace console {

/* Create the REPL and its commands. Deliberately not fatal at the call site: a
 * box that cannot start a console still has to boot and serve time. */
esp_err_t start();

}  // namespace console
