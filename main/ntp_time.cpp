#include "ntp_time.h"

#include <cstdio>
#include <cstring>
#include <ctime>

#include "esp_log.h"
#include "esp_sntp.h"
#include "lwip/ip_addr.h"

namespace {

const char *TAG = "ntp_time";

/* Stepping is the only sane option here: a smooth slew would take weeks to move
 * a 1970 clock to the present, and an NTP server must not serve a clock that is
 * still catching up. */
constexpr bool kSmoothSync = false;

void formatLocal(time_t when, char *buf, size_t len)
{
    struct tm tm_now;
    localtime_r(&when, &tm_now);
    strftime(buf, len, "%Y-%m-%dT%H:%M:%S%z", &tm_now);
}

}  // namespace

NtpTime &NtpTime::instance()
{
    static NtpTime ntp;
    return ntp;
}

esp_err_t NtpTime::begin()
{
    /* NETIF_SNTP_TIME_SYNC carries the stepped time, so /healthz and the logs can
     * say when the clock was last set instead of guessing from its value. */
    esp_err_t err = esp_event_handler_register(NETIF_SNTP_EVENT, ESP_EVENT_ANY_ID,
                                               &NtpTime::onSyncEvent, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cannot listen for time sync events: %s", esp_err_to_name(err));
        return err;
    }
    return applySettings();
}

esp_err_t NtpTime::applySettings()
{
    const Settings      &settings = Settings::instance();
    const char          *raw      = settings.get("timeSource");
    Settings::TimeSource wanted   = settings.timeSource();

    if (strcmp(raw, "off") != 0 && strcmp(raw, "dhcp") != 0 && strcmp(raw, "static") != 0) {
        ESP_LOGW(TAG, "unknown timeSource '%s', treating it as off", raw);
    }
    if (wanted == Settings::TimeSource::Static && settings.ntpServer()[0] == '\0') {
        ESP_LOGW(TAG, "timeSource=static needs ntpServer; leaving the clock alone");
        wanted = Settings::TimeSource::Off;
    }

    /* Cheap no-op when a POST /config touched something else. */
    if (m_running && wanted == m_source && strcmp(settings.ntpServer(), m_server) == 0) {
        return ESP_OK;
    }

    stop();
    return start(wanted, settings.ntpServer());
}

esp_err_t NtpTime::start(Settings::TimeSource source, const char *server)
{
    if (source == Settings::TimeSource::Off) {
        m_source = source;
        ESP_LOGI(TAG, "time source: off (the clock runs from reset)");
        return ESP_OK;
    }

    if (source == Settings::TimeSource::Static) {
        snprintf(m_server, sizeof(m_server), "%s", server);
    }

    esp_sntp_config_t config = {};
    config.smooth_sync      = kSmoothSync;
    config.server_from_dhcp = (source == Settings::TimeSource::Dhcp);
    config.wait_for_sync    = true; /* gives syncNow() a semaphore to wait on */
    config.start            = true;
    if (source == Settings::TimeSource::Static) {
        config.num_of_servers = 1;
        config.servers[0]     = m_server;
    }

    esp_err_t err = esp_netif_sntp_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sntp init: %s", esp_err_to_name(err));
        m_source = Settings::TimeSource::Off;
        return err;
    }

    m_source  = source;
    m_running = true;
    if (source == Settings::TimeSource::Dhcp) {
        ESP_LOGI(TAG, "time source: dhcp (upstream from option 42, once the lease lands)");
    } else {
        ESP_LOGI(TAG, "time source: static %s", m_server);
    }
    return ESP_OK;
}

void NtpTime::stop()
{
    if (m_running) {
        esp_netif_sntp_deinit();
        m_running = false;
    }
    m_source    = Settings::TimeSource::Off;
    m_server[0] = '\0';
}

void NtpTime::onGotIp()
{
    if (!m_running) {
        return;
    }

    char upstream[Settings::kStrLen];
    if (upstreamText(upstream, sizeof(upstream)) == ESP_OK) {
        ESP_LOGI(TAG, "upstream time server: %s", upstream);
    } else if (m_source == Settings::TimeSource::Dhcp) {
        ESP_LOGW(TAG, "no NTP server in the lease (option 42); "
                      "set timeSource=static to name one");
    }

    /* The poll started at boot may have gone out before the lease (and with it
     * the option-42 address) existed, so poll again now. */
    esp_err_t err = esp_netif_sntp_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "restart sntp: %s", esp_err_to_name(err));
    }
}

esp_err_t NtpTime::syncNow(TickType_t timeout)
{
    if (!m_running) {
        return ESP_ERR_INVALID_STATE;
    }
    /* start() is stop+init in the SNTP module: it sends a request immediately
     * rather than waiting out CONFIG_LWIP_SNTP_UPDATE_DELAY. */
    esp_err_t err = esp_netif_sntp_start();
    if (err != ESP_OK) {
        return err;
    }
    return esp_netif_sntp_sync_wait(timeout);
}

bool NtpTime::upstreamReachable() const
{
    unsigned int reachability = 0;
    if (esp_netif_sntp_reachability(0, &reachability) != ESP_OK) {
        return false;
    }
    return reachability != 0;
}

esp_err_t NtpTime::upstreamText(char *buf, size_t len) const
{
    if (buf == nullptr || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    buf[0] = '\0';
    if (!m_running) {
        return ESP_ERR_INVALID_STATE;
    }
    const char *name = esp_sntp_getservername(0);
    if (name != nullptr && name[0] != '\0') {
        snprintf(buf, len, "%s", name);
        return ESP_OK;
    }
    const ip_addr_t *addr = esp_sntp_getserver(0);
    if (addr == nullptr || ip_addr_isany(addr)) {
        return ESP_ERR_NOT_FOUND;
    }
    ipaddr_ntoa_r(addr, buf, static_cast<int>(len));
    return ESP_OK;
}

void NtpTime::onSyncEvent(void *arg, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)base;
    (void)event_id;

    NtpTime *self = static_cast<NtpTime *>(arg);
    const esp_netif_sntp_time_sync_t *sync =
        static_cast<const esp_netif_sntp_time_sync_t *>(event_data);

    time_t when = sync != nullptr ? static_cast<time_t>(sync->tv.tv_sec) : time(nullptr);
    self->m_last_sync.store(static_cast<long long>(when), std::memory_order_relaxed);

    char text[32];
    formatLocal(when, text, sizeof(text));

    char upstream[Settings::kStrLen];
    if (self->upstreamText(upstream, sizeof(upstream)) != ESP_OK) {
        snprintf(upstream, sizeof(upstream), "?");
    }
    ESP_LOGI(TAG, "clock set from %s: %s", upstream, text);
}
