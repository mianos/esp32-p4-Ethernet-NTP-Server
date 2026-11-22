#include "clock/ntp_time.h"

#include <cstdio>
#include <ctime>

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"

#include "clock/clock_source.h"
#include "lwip/ip_addr.h"
#include "net/net_state.h"
#include "app/text.h"

namespace {

/* Step, not slew: the clock may start at 1970. */
constexpr bool kSmoothSync = false;

/* lwIP's default of an hour leaves no margin within kUpstreamFreshS. */
constexpr uint32_t kPollIntervalMs = 60000;

/* lwIP's server 0 as text, empty when unset. lwIP only rewrites it on a DHCP
 * bind or an SNTP start, so it is stale after a stop(). */
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

bool haveLease()
{
    esp_netif_t        *netif = net_state::netif();
    esp_netif_ip_info_t info;
    return netif != nullptr && esp_netif_get_ip_info(netif, &info) == ESP_OK && info.ip.addr != 0;
}

/* lwIP's SNTP client resolves A records only, so an IPv6 literal cannot work;
 * it is reported rather than silently never syncing. */
bool looksLikeIpv6(const std::string &text)
{
    return text.find(':') != std::string::npos;
}

}  // namespace

std::string isoTimestamp(time_t when)
{
    struct tm local;
    localtime_r(&when, &local);

    /* strftime for %z. */
    char text[32];
    strftime(text, sizeof(text), "%Y-%m-%dT%H:%M:%S%z", &local);
    return text;
}

NtpTime &NtpTime::instance()
{
    static NtpTime ntp;
    return ntp;
}

namespace {
/* RAII around a FreeRTOS mutex; std::mutex pulls in std::system_error (5 KB). */
class MutexLock {
public:
    explicit MutexLock(SemaphoreHandle_t sem) : m_sem(sem)
    {
        if (m_sem != nullptr) {
            xSemaphoreTake(m_sem, portMAX_DELAY);
        }
    }
    ~MutexLock()
    {
        if (m_sem != nullptr) {
            xSemaphoreGive(m_sem);
        }
    }
    MutexLock(const MutexLock &)            = delete;
    MutexLock &operator=(const MutexLock &) = delete;

private:
    SemaphoreHandle_t m_sem;
};
}  // namespace

std::string NtpTime::serverCopy() const
{
    MutexLock lock(m_serverLock);
    return m_server;
}

void NtpTime::setServer(const std::string &server)
{
    MutexLock lock(m_serverLock);
    m_server = server;
}

esp_err_t NtpTime::begin()
{

    esp_err_t err =
        esp_event_handler_register(NETIF_SNTP_EVENT, ESP_EVENT_ANY_ID, &NtpTime::onSyncEvent, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cannot listen for time sync events: %s", esp_err_to_name(err));
        return err;
    }

    armDhcpServer();
    return applySettings();
}

Settings::TimeSource NtpTime::fallbackSource(std::string &server) const
{
    server = Settings::instance().ntpServer();
    if (!server.empty() && isSelfAddress(server)) {

        ESP_LOGW(TAG,
                 "ntpServer '%s' is this device; a zero offset against itself says "
                 "nothing, so the fallback uses the lease's server instead",
                 server.c_str());
        server.clear();
    }
    return server.empty() ? Settings::TimeSource::Dhcp : Settings::TimeSource::Static;
}

void NtpTime::armDhcpServer() const
{
#if LWIP_DHCP_GET_NTP_SRV
    /* Under gps/auto no client starts, so option 42 must be armed here for a
     * later fallback to have a server. It does not replay a lease already
     * processed; the next renewal brings the server. */
    const Settings::TimeSource source = Settings::instance().timeSource();
    if (source == Settings::TimeSource::Gps || source == Settings::TimeSource::Auto) {
        esp_sntp_servermode_dhcp(true);
    }
#endif
}

esp_err_t NtpTime::applySettings()
{
    MutexLock            lock(m_switchLock);
    const Settings      &settings = Settings::instance();
    const std::string   &text     = settings.timeSourceText();
    Settings::TimeSource wanted   = settings.timeSource();
    std::string          server;

    if (text != "off" && text != "dhcp" && text != "static" && text != "gps" && text != "auto") {
        ESP_LOGW(TAG, "unknown timeSource '%s', treating it as off", text.c_str());
    }
    armDhcpServer();

    /* Under "auto" ClockSource decides whether a client runs. */
    if (wanted == Settings::TimeSource::Auto) {
        wanted = ClockSource::instance().owner() == ClockOwner::Gps ? Settings::TimeSource::Gps
                                                                    : fallbackSource(server);
    } else if (wanted == Settings::TimeSource::Static) {
        server = settings.ntpServer();
    }

    if (wanted == Settings::TimeSource::Static && server.empty()) {
        ESP_LOGW(TAG, "timeSource=static needs ntpServer; leaving the clock alone");
        wanted = Settings::TimeSource::Off;
    }

    /* No-op when nothing relevant changed; only static mode compares the
     * server. */
    const bool running    = m_running.load(std::memory_order_relaxed);
    const bool sameSource = wanted == m_source.load(std::memory_order_relaxed);
    const bool sameServer = wanted != Settings::TimeSource::Static || server == serverCopy();
    if (running && sameSource && sameServer) {
        return ESP_OK;
    }

    stop();
    return start(wanted, server);
}

bool NtpTime::isSelfAddress(const std::string &name)
{
    esp_netif_t *netif = net_state::netif();
    if (netif == nullptr) {
        return false;
    }
    esp_netif_ip_info_t ip = {};
    if (esp_netif_get_ip_info(netif, &ip) != ESP_OK || ip.ip.addr == 0) {
        return false;
    }
    char self[16];
    std::snprintf(self, sizeof(self), "%u.%u.%u.%u", ip4_addr1(&ip.ip), ip4_addr2(&ip.ip),
                  ip4_addr3(&ip.ip), ip4_addr4(&ip.ip));
    return name == self;
}

esp_err_t NtpTime::startFallback()
{
    MutexLock lock(m_switchLock);
    if (m_running.load(std::memory_order_relaxed)) {
        /* Already running: just poll. */
        return esp_netif_sntp_start();
    }

    armDhcpServer();

    std::string          server;
    Settings::TimeSource source = fallbackSource(server);
    if (source == Settings::TimeSource::Dhcp) {

        server = lwipServer();
        if (server.empty()) {
            ESP_LOGW(TAG, "the current lease carries no option-42 server: the fallback has "
                          "nothing to poll until it renews (set ntpServer to name one)");
        }
    }
    ESP_LOGW(TAG, "starting the fallback upstream client (%s %s)", Settings::timeSourceName(source),
             server.c_str());
    return start(source, server);
}

void NtpTime::stopFallback()
{
    MutexLock lock(m_switchLock);
    if (!m_running.load(std::memory_order_relaxed)) {
        return;
    }
    ESP_LOGI(TAG, "stopping the fallback upstream client");
    stop();
}

esp_err_t NtpTime::start(Settings::TimeSource source, const std::string &server)
{
    if (source == Settings::TimeSource::Gps) {
        /* No client: the GPS owns CLOCK_REALTIME. m_source is still recorded
         * for /healthz. */
        m_source.store(source, std::memory_order_relaxed);
        ESP_LOGI(TAG, "time source: gps (u-blox over UART, PPS on GPIO14)");
        return ESP_OK;
    }

    if (source == Settings::TimeSource::Off) {
        m_source.store(source, std::memory_order_relaxed);
        ESP_LOGI(TAG, "time source: off (the clock runs from reset)");
        return ESP_OK;
    }

    /* Kept for the refid and /healthz; in dhcp mode lwIP polls its own
     * table. */
    setServer(server);
    if (source == Settings::TimeSource::Static) {
        if (looksLikeIpv6(server)) {
            ESP_LOGW(TAG,
                     "ntpServer '%s' is an IPv6 address; lwIP's SNTP client "
                     "resolves A records only, so it cannot sync from it -- "
                     "use an IPv4 address or a host name",
                     server.c_str());
        }
    }

    esp_sntp_config_t config = {};
    config.smooth_sync       = kSmoothSync;
    config.server_from_dhcp  = (source == Settings::TimeSource::Dhcp);
    config.wait_for_sync     = true; /* gives syncNow() a semaphore to wait on */
    config.start             = true;
    if (source == Settings::TimeSource::Static) {
        /* lwIP keeps this pointer; see m_lwipServer. */
        const std::string_view name = server;
        const std::size_t      n    = std::min(name.size(), sizeof(m_lwipServer) - 1);
        name.copy(m_lwipServer, n);
        m_lwipServer[n]       = '\0';
        config.num_of_servers = 1;
        config.servers[0]     = m_lwipServer;
    }

    esp_sntp_set_sync_interval(kPollIntervalMs);

    esp_err_t err = esp_netif_sntp_init(&config);
    if (err != ESP_OK) {

        ESP_LOGE(TAG, "sntp init: %s -- the clock is not being polled", esp_err_to_name(err));
        m_source.store(Settings::TimeSource::Off, std::memory_order_relaxed);
        return err;
    }

    m_source.store(source, std::memory_order_relaxed);
    m_running.store(true, std::memory_order_relaxed);
    if (source == Settings::TimeSource::Dhcp) {
        ESP_LOGI(TAG, "time source: dhcp (upstream from option 42, once the lease lands)");
        /* With a lease already held, option 42 arrives at the next renewal. */
        if (serverCopy().empty() && haveLease()) {
            ESP_LOGW(TAG, "no server in the current lease yet: option 42 arrives at "
                          "the next DHCP bind (a reboot applies it now)");
        }
    } else {
        ESP_LOGI(TAG, "time source: static %s", server.c_str());
    }
    return ESP_OK;
}

void NtpTime::stop()
{
    if (m_running.load(std::memory_order_relaxed)) {
        esp_netif_sntp_deinit();
        m_running.store(false, std::memory_order_relaxed);
    }
    /* Otherwise a new fallback would count as synced before its first answer. */
    m_last_sync.store(0, std::memory_order_relaxed);
    m_source.store(Settings::TimeSource::Off, std::memory_order_relaxed);
    setServer({});
}

void NtpTime::onGotIp()
{
    /* Read the leased server now, while lwIP's table is current, even if no
     * client is running yet. */
    const Settings::TimeSource wanted = Settings::instance().timeSource();
    if (wanted == Settings::TimeSource::Dhcp || wanted == Settings::TimeSource::Auto) {
        setServer(lwipServer());
    }

    if (!m_running.load(std::memory_order_relaxed)) {
        return;
    }

    const std::string upstream = serverCopy();
    if (upstream.empty()) {
        ESP_LOGW(TAG, "no NTP server in the lease (option 42); "
                      "set timeSource=static to name one");
    } else {
        ESP_LOGI(TAG, "upstream time server: %s", upstream.c_str());
    }

    /* The boot poll may have preceded the lease. */
    esp_err_t err = esp_netif_sntp_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "restart sntp: %s", esp_err_to_name(err));
    }
}

esp_err_t NtpTime::syncNow(TickType_t timeout)
{
    MutexLock lock(m_switchLock);
    if (!m_running.load(std::memory_order_relaxed)) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Restarting sends a request at once. */
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
    std::string server = serverCopy();
    if (server.empty()) {
        /* In dhcp mode a client started after the lease has no copy; read
         * lwIP's table. */
        server = lwipServer();
    }
    if (!m_running.load(std::memory_order_relaxed) || server.empty()) {
        return std::nullopt;
    }
    return server;
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
