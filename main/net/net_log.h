/* Console output over the network. See net_log.cpp for why it exists and how
 * it avoids blocking the logger. */
#pragma once

#include "esp_err.h"

namespace net_log {

/* Starts the sink if a host and port are configured. Safe to call more than
 * once; the hook is installed once. Returns ESP_OK when nothing is configured,
 * because an unconfigured sink is not a failure. */
esp_err_t start();

}  // namespace net_log
