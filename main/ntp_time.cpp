#include "ntp_time.h"

#include <cstdio>
#include <ctime>

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "lwip/ip_addr.h"

#include "net_state.h"

namespace {

/* Stepping is the only sane option here: a smooth slew would take weeks to move
 * a 1970 clock to the present, and an NTP server must not serve a clock that is
 * still catching up. */
constexpr bool kSmoothSync = false;

/* The server lwIP holds under index 0, as text; empty when unset. Note that
 * lwIP only rewrites that entry on a DHCP bind or an SNTP start, so it stays
 * stale across a stop() — read it at the moment it is known to be current. */
std::string lwipServer()
{
    const ip_addr_t *addr = esp_sntp_getserver(0);
    if (addr == nullptr || ip_addr_isany(addr)) {
        return {};
    }
    char text[IPADDR_STRLEN_MAX] = {};
    ipaddr_ntoa_r(addr, text, sizeof(text));
    return text;
}

/* True once the netif has an address, i.e. a mode switch rather than boot. */
bool haveLease()
{
    esp_netif_t        *netif = net_state::netif();
    esp_netif_ip_info_t info;
    return netif != nullptr && esp_netif_get_ip_info(netif, &info) == ESP_OK &&
           info.ip.addr != 0;
}

/* lwIP's SNTP client resolves its server with dns_gethostbyname(), i.e. A
 * records only, so an IPv6 literal can never work however it is written: the
 * name goes to DNS, and DNS here only ever answers with an IPv4 address.
 * Catching it turns a silent "never syncs" into one line in the log. */
bool looksLikeIpv6(const std::string &text)
{
    return text.find(':') != std::string::npos;
}

}  // namespace

std::string isoTimestamp(time_t when)
{
    struct tm local;
    localtime_r(&when, &local);

    /* strftime is the only formatter in the C library that knows about %z, and
     * it writes to a buffer; 32 bytes holds any output it can produce here. */
    char text[32];
    strftime(text, sizeof(text), "%Y-%m-%dT%H:%M:%S%z", &local);
    return text;
}

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
    const std::string   &text     = settings.timeSourceText();
    Settings::TimeSource wanted   = settings.timeSource();

    if (text != "off" && text != "dhcp" && text != "static") {
        ESP_LOGW(TAG, "unknown timeSource '%s', treating it as off", text.c_str());
    }
    if (wanted == Settings::TimeSource::Static && settings.ntpServer().empty()) {
        ESP_LOGW(TAG, "timeSource=static needs ntpServer; leaving the clock alone");
        wanted = Settings::TimeSource::Off;
    }

    /* Cheap no-op when a POST /config touched something else. In dhcp mode the
     * server comes from the lease, so only static mode compares it. */
    if (m_running && wanted == m_source &&
        (wanted != Settings::TimeSource::Static || settings.ntpServer() == m_server)) {
        return ESP_OK;
    }

    stop();
    return start(wanted, settings.ntpServer());
}

esp_err_t NtpTime::start(Settings::TimeSource source, const std::string &server)
{
    if (source == Settings::TimeSource::Off) {
        m_source = source;
        ESP_LOGI(TAG, "time source: off (the clock runs from reset)");
        return ESP_OK;
    }

    if (source == Settings::TimeSource::Static) {
        m_server = server;
        if (looksLikeIpv6(m_server)) {
            ESP_LOGW(TAG, "ntpServer '%s' is an IPv6 address; lwIP's SNTP client "
                          "resolves A records only, so it cannot sync from it -- "
                          "use an IPv4 address or a host name",
                     m_server.c_str());
        }
    }

    /* esp_netif_sntp_init() copies these names into lwIP, so m_server only has
     * to outlive the call — it is a member anyway. */
    esp_sntp_config_t config = {};
    config.smooth_sync      = kSmoothSync;
    config.server_from_dhcp = (source == Settings::TimeSource::Dhcp);
    config.wait_for_sync    = true; /* gives syncNow() a semaphore to wait on */
    config.start            = true;
    if (source == Settings::TimeSource::Static) {
        config.num_of_servers = 1;
        config.servers[0]     = m_server.c_str();
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
        /* lwIP only hands the option-42 address over on a DHCP bind, so switching
         * to "dhcp" with a lease already in hand waits for the next renewal. */
        if (m_server.empty() && haveLease()) {
            ESP_LOGW(TAG, "no server in the current lease yet: option 42 arrives at "
                          "the next DHCP bind (a reboot applies it now)");
        }
    } else {
        ESP_LOGI(TAG, "time source: static %s", m_server.c_str());
    }
    return ESP_OK;
}

void NtpTime::stop()
{
    if (m_running) {
        esp_netif_sntp_deinit();
        m_running = false;
    }
    m_source = Settings::TimeSource::Off;
    m_server.clear();
}

void NtpTime::onGotIp()
{
    if (!m_running) {
        return;
    }

    if (m_source == Settings::TimeSource::Dhcp) {
        /* The ACK has just been applied to lwIP's SNTP table (option 42 arrives
         * as an address, not a name), so this is when it is worth reading. */
        m_server = lwipServer();
    }

    if (m_server.empty()) {
        ESP_LOGW(TAG, "no NTP server in the lease (option 42); "
                      "set timeSource=static to name one");
    } else {
        ESP_LOGI(TAG, "upstream time server: %s", m_server.c_str());
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

std::optional<std::string> NtpTime::upstreamName() const
{
    if (!m_running || m_server.empty()) {
        return std::nullopt;
    }
    return m_server;
}

void NtpTime::onSyncEvent(void *arg, esp_event_base_t, int32_t, void *event_data)
{
    auto       *self = static_cast<NtpTime *>(arg);
    const auto *sync = static_cast<const esp_netif_sntp_time_sync_t *>(event_data);

    const time_t when = sync != nullptr ? static_cast<time_t>(sync->tv.tv_sec) : time(nullptr);
    self->m_last_sync.store(static_cast<long long>(when), std::memory_order_relaxed);

    const std::string upstream  = self->upstreamName().value_or("?");
    const std::string timestamp = isoTimestamp(when);
    ESP_LOGI(TAG, "clock set from %s: %s", upstream.c_str(), timestamp.c_str());
}
