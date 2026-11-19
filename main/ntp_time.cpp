#include "ntp_time.h"

#include <cstdio>
#include <ctime>

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"

#include "clock_source.h"
#include "lwip/ip_addr.h"
#include "net_state.h"
#include "text.h"

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
    MutexLock(const MutexLock &) = delete;
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
    /* NETIF_SNTP_TIME_SYNC carries the stepped time, so /healthz and the logs can
     * say when the clock was last set instead of guessing from its value. */
    esp_err_t err = esp_event_handler_register(NETIF_SNTP_EVENT, ESP_EVENT_ANY_ID,
                                               &NtpTime::onSyncEvent, this);
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
        /* Worth catching: an offset measured against ourselves is always zero,
         * so the clock would look disciplined and free-run for ever. */
        ESP_LOGW(TAG, "ntpServer '%s' is this device; a zero offset against itself says "
                      "nothing, so the fallback uses the lease's server instead",
                 server.c_str());
        server.clear();
    }
    return server.empty() ? Settings::TimeSource::Dhcp : Settings::TimeSource::Static;
}

void NtpTime::armDhcpServer() const
{
#if LWIP_DHCP_GET_NTP_SRV
    /* Under gps/auto nothing starts the client, so nothing would call
     * sntp_servermode_dhcp() and lwIP would reject option 42 as the ACK is
     * processed -- leaving a fallback engaged later with no server to use. The
     * flag only makes the lease's server collectable; no timer runs until a
     * client does.
     *
     * Arming after a lease has already been processed does not replay that
     * lease, so this runs at boot and again whenever the setting moves to a
     * source that might need it; the next renewal collects the server. */
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

    if (text != "off" && text != "dhcp" && text != "static" && text != "gps" &&
        text != "auto") {
        ESP_LOGW(TAG, "unknown timeSource '%s', treating it as off", text.c_str());
    }
    armDhcpServer();

    /* "auto" is not a client mode: what the client should be doing depends on
     * the receiver, and clock_source.cpp is the one place that knows. Asking it
     * here keeps a single answer to "who owns the clock" instead of a second
     * opinion that could disagree with the servo. */
    if (wanted == Settings::TimeSource::Auto) {
        wanted = ClockSource::instance().owner() == ClockOwner::Gps
                     ? Settings::TimeSource::Gps
                     : fallbackSource(server);
    } else if (wanted == Settings::TimeSource::Static) {
        server = settings.ntpServer();
    }

    if (wanted == Settings::TimeSource::Static && server.empty()) {
        ESP_LOGW(TAG, "timeSource=static needs ntpServer; leaving the clock alone");
        wanted = Settings::TimeSource::Off;
    }

    /* Cheap no-op when a POST /config touched something else. In dhcp mode the
     * server comes from the lease, so only static mode compares it. */
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
        /* Already up: nudge the poll rather than rebuild the client, so a retry
         * does not drop a server address that is working. */
        return esp_netif_sntp_start();
    }

    armDhcpServer();

    std::string          server;
    Settings::TimeSource source = fallbackSource(server);
    if (source == Settings::TimeSource::Dhcp) {
        /* The name only: lwIP polls its own server table, filled from the lease. */
        server = lwipServer();
        if (server.empty()) {
            ESP_LOGW(TAG, "the current lease carries no option-42 server: the fallback has "
                          "nothing to poll until it renews (set ntpServer to name one)");
        }
    }
    ESP_LOGW(TAG, "starting the fallback upstream client (%s %s)",
             Settings::timeSourceName(source), server.c_str());
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

    /* The name is kept for the refid and /healthz in both modes; in dhcp mode
     * lwIP's own table is what actually gets polled. */
    setServer(server);
    if (source == Settings::TimeSource::Static) {
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
         * the previous contents is already gone. The array therefore stays; the
         * copy into it is a sized one, which is the part snprintf was doing. */
        const std::string_view name = server;
        const std::size_t      n    = std::min(name.size(), sizeof(m_lwipServer) - 1);
        name.copy(m_lwipServer, n);
        m_lwipServer[n] = '\0';
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
    /* Taken before the running check: the leased name is what the refid and
     * /healthz report, and under gps/auto no client is running yet at the first
     * lease. The ACK has just been applied to lwIP's SNTP table (option 42
     * arrives as an address, not a name), so this is when it is worth reading. */
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

    /* The poll started at boot may have gone out before the lease (and with it
     * the option-42 address) existed, so poll again now. */
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
    std::string server = serverCopy();
    if (server.empty()) {
        /* The name normally lives here, but in dhcp mode it lives in lwIP's
         * table and m_server is only ever a copy of it. A client that started
         * after the lease -- a fallback, precisely -- never took the copy, and
         * without this read the reply's refid falls back to INIT: a client reads
         * that as "not synchronised" and demotes a server that is in fact
         * synced. Reading the table costs a format on the reply path and only
         * while the copy is empty. */
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
