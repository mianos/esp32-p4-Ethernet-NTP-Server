/* The serial console, the way back in when the network is not working: set
 * a static address, return to DHCP, reset the config, show status. Changes go
 * through Settings::applyJson(), the same validation as POST /config. */
#pragma once

#include "esp_err.h"

namespace console {

/* Not fatal at the call site: the box must boot without it. */
esp_err_t start();

}  // namespace console
