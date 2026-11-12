/*
 * esp32p4-ntp -- the boot sequence, and nothing else.
 *
 * Board: Waveshare ESP32-P4-ETH (ESP32-P4NRW32 + IP101GRI 10/100 PHY on RMII).
 * The RJ45 is the only interface the P4 has -- no radio -- so everything
 * field-facing hangs off it: the DHCP lease, the clock the NTP responder
 * serves, and the HTTP surface that configures it.
 *
 * The link itself (pin map, EMAC + PHY, netif, event handlers, PHY diagnostics,
 * the status task) is EthernetLink's, in eth_link.h/.cpp. What is left here is
 * the order those parts come up in, which is the part that is easy to get wrong:
 *
 *   1. NVS. Settings live there; a stale blob from an older layout is erased
 *      rather than migrated.
 *   2. Settings. Hostname (DHCP option 12), the upstream NTP server, the test
 *      mode flag. Read from NVS, so it has to follow 1.
 *   3. esp_netif_init() and the default event loop. EthernetLink registers
 *      handlers on that loop and creates its netif on that stack, so both come
 *      first.
 *   4. GpsTime::begin(). UART1 and the PPS interrupt: the stratum-1 reference,
 *      armed before anything asks it for the time. It needs no network, so it
 *      can come first, and a board with no receiver on those pins simply never
 *      locks instead of failing here.
 *   5. NtpTime::begin(). Arms the SNTP client before the first DHCP DISCOVER:
 *      lwIP only keeps an option-42 NTP server if sntp_servermode_dhcp() is on
 *      when the ACK is processed. It yields the clock to the GPS when
 *      timeSource is "gps".
 *   6. EthernetLink::begin(). Driver, PHY diagnostic, netif, handlers, status
 *      task, esp_eth_start(). Everything after that is event-driven: link up,
 *      the lease (which starts the web server and the responder and confirms an
 *      OTA'd image), then IPv6 addresses as DAD and SLAAC deliver them.
 *
 * A failure on this path is not recoverable -- there is no console to type a
 * second command into -- so ESP_ERROR_CHECK aborts with the file and line, which
 * is what the README's troubleshooting section points at.
 */

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "eth_link.h"
#include "gps_time.h"
#include "ntp_time.h"
#include "settings.h"

namespace {

/* The same tag as eth_link.cpp: one device, one log prefix, whichever file
 * emits the line. */
constexpr const char *TAG = "eth_demo";

/* Settings live in NVS. A stale blob from an older layout is erased rather than
 * carried forward: the defaults are the documented starting point. */
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
    ESP_LOGI(TAG, "esp32p4-ntp: RMII + IP101, DHCP client, HTTP /healthz, NTP from GPS/PPS or upstream");

    initNvs();
    Settings::instance().begin();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* The receiver's UART and PPS line first: a fix takes tens of seconds to
     * settle, so the sooner it is listening the sooner the clock is real. */
    ESP_ERROR_CHECK(GpsTime::instance().begin());

    /* Armed before the first DHCP DISCOVER: lwIP only keeps an option-42 NTP
     * server if sntp_servermode_dhcp() is on when the ACK is processed. */
    ESP_ERROR_CHECK(NtpTime::instance().begin());

    ESP_ERROR_CHECK(EthernetLink::instance().begin());
}
