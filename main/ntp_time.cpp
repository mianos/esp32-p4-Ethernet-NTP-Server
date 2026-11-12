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

/* How often the client re-polls once the clock is set. lwIP defaults to an hour,
 * which leaves the responder's freshness window no margin at all: one lost reply
 * would have the server advertising stratum 16 for a whole cycle. */
constexpr uint32_t kPollIntervalMs = 60000;

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

/* The upstream name is read on the responder task and written on two others, so
 * every access goes through the lock -- see the note in ntp_time.h. */
namespace {
/* RAII around the FreeRTOS mutex. std::mutex would read better, but its failure
 * path drags std::system_error into the image for 5 KB, and there is nothing to
 * recover from here: a lock that cannot be taken is already fatal. */
class ServerLock {
public:
    explicit ServerLock(SemaphoreHandle_t sem) : m_sem(sem)
    {
        if (m_sem != nullptr) {
            xSemaphoreTake(m_sem, portMAX_DELAY);
        }
    }
    ~ServerLock()
    {
        if (m_sem != nullptr) {
            xSemaphoreGive(m_sem);
        }
    }
    ServerLock(const ServerLock &) = delete;
    ServerLock &operator=(const ServerLock &) = delete;

private:
    SemaphoreHandle_t m_sem;
};
}  // namespace

std::string NtpTime::serverCopy() const
{
    ServerLock lock(m_serverLock);
    return m_server;
}

void NtpTime::setServer(const std::string &server)
{
    ServerLock lock(m_serverLock);
    m_server = server;
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

    if (text != "off" && text != "dhcp" && text != "static" && text != "gps") {
        ESP_LOGW(TAG, "unknown timeSource '%s', treating it as off", text.c_str());
    }
    if (wanted == Settings::TimeSource::Static && settings.ntpServer().empty()) {
        ESP_LOGW(TAG, "timeSource=static needs ntpServer; leaving the clock alone");
        wanted = Settings::TimeSource::Off;
    }

    /* Cheap no-op when a POST /config touched something else. In dhcp mode the
     * server comes from the lease, so only static mode compares it. */
    const bool running    = m_running.load(std::memory_order_relaxed);
    const bool sameSource = wanted == m_source.load(std::memory_order_relaxed);
    const bool sameServer =
        wanted != Settings::TimeSource::Static || settings.ntpServer() == serverCopy();
    if (running && sameSource && sameServer) {
        return ESP_OK;
    }

    stop();
    return start(wanted, settings.ntpServer());
}

esp_err_t NtpTime::start(Settings::TimeSource source, const std::string &server)
{
    if (source == Settings::TimeSource::Gps) {
        /* The clock is disciplined by gps_time.cpp, not by a poll. There is no
         * client to start, and starting one would put a second owner on
         * CLOCK_REALTIME -- SNTP steps, the PPS slews, and the two undo each
         * other. m_source is still recorded, so /healthz names the source the
         * device is actually using. */
        m_source.store(source, std::memory_order_relaxed);
        ESP_LOGI(TAG, "time source: gps (u-blox over UART, PPS on GPIO14)");
        return ESP_OK;
    }

    if (source == Settings::TimeSource::Off) {
        m_source.store(source, std::memory_order_relaxed);
        ESP_LOGI(TAG, "time source: off (the clock runs from reset)");
        return ESP_OK;
    }

    if (source == Settings::TimeSource::Static) {
        setServer(server);
        if (looksLikeIpv6(server)) {
            ESP_LOGW(TAG, "ntpServer '%s' is an IPv6 address; lwIP's SNTP client "
                          "resolves A records only, so it cannot sync from it -- "
                          "use an IPv4 address or a host name",
                     server.c_str());
        }
    }

    esp_sntp_config_t config = {};
    config.smooth_sync      = kSmoothSync;
    config.server_from_dhcp = (source == Settings::TimeSource::Dhcp);
    config.wait_for_sync    = true; /* gives syncNow() a semaphore to wait on */
    config.start            = true;
    if (source == Settings::TimeSource::Static) {
        /* lwIP keeps this pointer (sntp_setservername() does not copy), so it is
         * the buffer that is handed over, not a std::string another task could
         * reallocate. start() only runs after stop(), so the client that read
         * the previous contents is already gone. */
        std::snprintf(m_lwipServer, sizeof(m_lwipServer), "%s", server.c_str());
        config.num_of_servers = 1;
        config.servers[0]     = m_lwipServer;
    }

    esp_sntp_set_sync_interval(kPollIntervalMs);

    esp_err_t err = esp_netif_sntp_init(&config);
    if (err != ESP_OK) {
        /* The stored setting stays as it was written: /healthz then reports
         * ntp_source "off" while the clock is not being polled at all. */
        ESP_LOGE(TAG, "sntp init: %s -- the clock is not being polled", esp_err_to_name(err));
        m_source.store(Settings::TimeSource::Off, std::memory_order_relaxed);
        return err;
    }

    m_source.store(source, std::memory_order_relaxed);
    m_running.store(true, std::memory_order_relaxed);
    if (source == Settings::TimeSource::Dhcp) {
        ESP_LOGI(TAG, "time source: dhcp (upstream from option 42, once the lease lands)");
        /* lwIP only hands the option-42 address over on a DHCP bind, so switching
         * to "dhcp" with a lease already in hand waits for the next renewal. */
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
    m_source.store(Settings::TimeSource::Off, std::memory_order_relaxed);
    setServer({});
}

void NtpTime::onGotIp()
{
    if (!m_running.load(std::memory_order_relaxed)) {
        return;
    }

    if (m_source.load(std::memory_order_relaxed) == Settings::TimeSource::Dhcp) {
        /* The ACK has just been applied to lwIP's SNTP table (option 42 arrives
         * as an address, not a name), so this is when it is worth reading. */
        setServer(lwipServer());
    }

    const std::string upstream = serverCopy();
    if (upstream.empty()) {
        ESP_LOGW(TAG, "no NTP server in the lease (option 42); "
                      "set timeSource=static to name one");
    } else {
        ESP_LOGI(TAG, "upstream time server: %s", upstream.c_str());
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
    if (!m_running.load(std::memory_order_relaxed)) {
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
    const std::string server = serverCopy();
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
