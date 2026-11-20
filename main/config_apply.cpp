#include "config_apply.h"

#include "esp_log.h"
#include "esp_netif.h"

#include "clock_source.h"
#include "eth_link.h"
#include "net_log.h"
#include "net_state.h"
#include "ntp_server.h"
#include "ntp_time.h"
#include "settings.h"

namespace {

constexpr const char *TAG = "config";

}  // namespace

void config_apply::hostname(const std::string &name)
{
    esp_netif_t *netif = net_state::netif();
    if (netif == nullptr) {
        return;
    }

    const esp_err_t err = esp_netif_set_hostname(netif, name.c_str());
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set hostname '%s': %s", name.c_str(), esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "hostname now %s (DHCP sends it on the next lease)", name.c_str());
}

void config_apply::afterChange(const std::string &hostnameBefore, bool logChanged)
{
    Settings &settings = Settings::instance();

    /* The sink first: a changed logHost is usually the reason the box is being
     * reconfigured at all, and it is harmless if nothing else changed. */
    if (logChanged) {
        net_log::start();
    }
    if (settings.hostname() != hostnameBefore) {
        hostname(settings.hostname());
    }

    /* Addressing before the clock: a new address is what makes the box
     * reachable again, and the responders bind to all addresses anyway. */
    EthernetLink::instance().applySettings();

    /* The arbiter goes before the client: leaving "auto" has to hand the clock
     * back before NtpTime decides whether a client should be running at all. */
    ClockSource::instance().applySettings();
    NtpTime::instance().applySettings();
    /* And so does serveNtp: off stops the responder, on starts it. */
    NtpServer::instance().applySettings();
}
