/*
 * The upstream SNTP client, which steps the system clock:
 *
 *   timeSource = "dhcp"    server from DHCP option 42
 *   timeSource = "static"  Settings::ntpServer()
 *   timeSource = "gps"     no client; the receiver owns the clock
 *   timeSource = "auto"    no client until ClockSource starts a fallback
 *                          (ntpServer, else option 42)
 *   timeSource = "off"     no client
 *
 * lwIP only accepts option 42 if sntp_servermode_dhcp() is set before the DHCP
 * ACK, so begin() runs before esp_eth_start().
 */
#pragma once

#include <cstddef>
#include <ctime>

#include <atomic>
#include <optional>
#include <string>

#include "esp_event.h"
#include "esp_netif_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "app/settings.h"

/* Local ISO-8601 time, e.g. "2026-11-11T13:43:22+1100". */
std::string isoTimestamp(time_t when);

class NtpTime {
public:
    static NtpTime &instance();

    /* Register for sync events and apply the source. Call once at boot, before
     * the first DHCP DISCOVER (see above). */
    esp_err_t begin();

    /* Start the client for a fallback: ntpServer(), or option 42 when that is
     * unset or names this device. ESP_OK with neither leaves a client with
     * nothing to poll. */
    esp_err_t startFallback();

    void stopFallback();

    /* Re-read Settings and restart the client if the source changed. */
    esp_err_t applySettings();

    /* A lease arrived or was renewed: poll the upstream server straight away. */
    void onGotIp();

    /* Force a poll and wait for the result: ESP_OK once the clock is set,
     * ESP_ERR_TIMEOUT if it did not happen within `timeout`, ESP_ERR_INVALID_STATE
     * when there is no active time source. */
    esp_err_t syncNow(TickType_t timeout);

    bool running() const { return m_running.load(std::memory_order_relaxed); }

    /* True once this client has set the clock from an upstream server. */
    bool synced() const { return m_last_sync.load(std::memory_order_relaxed) != 0; }

    /* Epoch seconds of the last sync, 0 if never. */
    time_t lastSync() const
    {
        return static_cast<time_t>(m_last_sync.load(std::memory_order_relaxed));
    }

    /* The SNTP reachability register is non-zero. */
    bool upstreamReachable() const;

    /* Current upstream server as text ("192.0.2.1" or a hostname); nullopt
     * while the client has no server or is not running. */
    std::optional<std::string> upstreamName() const;

    std::string sourceName() const
    {
        return Settings::timeSourceName(m_source.load(std::memory_order_relaxed));
    }
    Settings::TimeSource source() const { return m_source.load(std::memory_order_relaxed); }

private:
    static constexpr const char *TAG = "ntp_time";

    NtpTime() = default;

    esp_err_t start(Settings::TimeSource source, const std::string &server);
    void      stop();

    /* The server and mechanism a fallback would use. */
    Settings::TimeSource fallbackSource(std::string &server) const;

    /* Let lwIP keep a leased option-42 server (see begin()). */
    void armDhcpServer() const;

    /* Whether a dotted quad is one of our addresses: polling ourselves gives a
     * zero offset for ever. Host names are not resolved. */
    static bool isSelfAddress(const std::string &name);

    static void onSyncEvent(void *arg, esp_event_base_t base, int32_t event_id, void *event_data);

    /* The server name is read by the responder and written by the HTTP and
     * event tasks, so it is behind m_serverLock. */
    std::string serverCopy() const;
    void        setServer(const std::string &server);

    /* sntp_setservername() keeps the pointer, so it gets m_lwipServer, written
     * only in start() after stop(). */
    static constexpr size_t kMaxServerName = 64;

    std::atomic<Settings::TimeSource> m_source{Settings::TimeSource::Off};
    std::atomic<bool>                 m_running{false};
    mutable SemaphoreHandle_t m_serverLock = xSemaphoreCreateMutex();
    /* Serialises start() and stop(), called from the HTTP and GPS tasks;
     * esp_netif_sntp_init() fails over a running client. */
    mutable SemaphoreHandle_t m_switchLock = xSemaphoreCreateMutex();
    std::string               m_server; /* guarded by m_serverLock */
    char                      m_lwipServer[kMaxServerName] = {};
    std::atomic<long long>    m_last_sync{0};
};
