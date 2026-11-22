/* esp32p4-ntp boot sequence. Board: Waveshare ESP32-P4-ETH (IP101GRI PHY, RMII).
 *
 * Order matters:
 *   1. NVS, then Settings, which are stored there.
 *   2. The serial console, before anything that can hang.
 *   3. esp_netif and the default event loop, which EthernetLink needs.
 *   4. ClockSource, whose quiet timer must run from boot.
 *   5. GpsTime: UART1 and the PPS interrupt; needs no network.
 *   6. NtpTime, before the first DHCP DISCOVER (option 42).
 *   7. EthernetLink. The rest is event driven: link, address (web server, NTP
 *      responder, PTP), IPv6.
 *
 * A failure here aborts via ESP_ERROR_CHECK with file and line. */

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "clock/clock_source.h"
#include "app/console.h"
#include "net/eth_link.h"
#include "gps/gps_time.h"
#include "clock/ntp_time.h"
#include "app/settings.h"

namespace {

/* Shares eth_link.cpp's tag. */
constexpr const char *TAG = "eth_demo";

/* A stale NVS layout is erased rather than migrated. */
void initNvs()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "erasing nvs partition (%s)", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

}  // namespace

extern "C" void app_main()
{
    ESP_LOGI(TAG,
             "esp32p4-ntp: RMII + IP101, DHCP client, HTTP /healthz, NTP from GPS/PPS or upstream");

    initNvs();
    Settings::instance().begin();

    /* First, so a bad stored address or a dead link can still be fixed. Not
     * fatal. */
    if (console::start() != ESP_OK) {
        ESP_LOGW(TAG, "console unavailable; continuing without it");
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ClockSource::instance().begin();

    ESP_ERROR_CHECK(GpsTime::instance().begin());

    ESP_ERROR_CHECK(NtpTime::instance().begin());

    ESP_ERROR_CHECK(EthernetLink::instance().begin());
}
