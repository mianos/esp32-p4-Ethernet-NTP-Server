#include "app/config_apply.h"

#include "esp_log.h"
#include "esp_netif.h"

#include "clock/clock_source.h"
#include "net/eth_link.h"
#include "net/net_log.h"
#include "net/net_state.h"
#include "ntp/ntp_server.h"
#include "clock/ntp_time.h"
#include "ptp/ptp.h"
#include "app/settings.h"
#include "app/settings.h"

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

    if (logChanged) {
        net_log::start();
    }
    if (settings.hostname() != hostnameBefore) {
        hostname(settings.hostname());
    }

    /* Addressing before the clock: it is what makes the box reachable again. */
    EthernetLink::instance().applySettings();

    /* ClockSource before NtpTime, so leaving "auto" hands the clock back first. */
    ClockSource::instance().applySettings();
    NtpTime::instance().applySettings();
    NtpServer::instance().applySettings();
    /* Domain and priority1 are passed to a running PTP task directly. */
    ptp::setIdentity(settings.ptpDomain(), settings.ptpPriority1());
    const std::string &mode   = settings.ptpMode();
    ptp::Mode          wanted = ptp::Mode::Off;
    if (mode == "master") {
        wanted = ptp::Mode::Master;
    } else if (mode == "slave") {
        wanted = ptp::Mode::Slave;
    }
    if (wanted != ptp::mode()) {
        (void)ptp::start(wanted, settings.ptpDomain(), settings.ptpPriority1());
    }
}
