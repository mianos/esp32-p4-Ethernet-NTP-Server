/* Log output over UDP; see net_log.cpp. */
#pragma once

#include "esp_err.h"

namespace net_log {

/* Start, retarget or stop the sink from the settings. Safe to call again;
 * ESP_OK when nothing is configured. */
esp_err_t start();

}  // namespace net_log
