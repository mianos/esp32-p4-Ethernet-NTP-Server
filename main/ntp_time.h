/*
 * Test-mode clock source.
 *
 * The P4 has no RTC and nothing disciplines the clock yet, so the firmware's
 * "now" is 1970 + uptime — which is fine for /healthz (it says so) but useless
 * to an NTP server. This class steps the system clock from an upstream SNTP
 * server so the responder (next milestone) has a real time to serve:
 *
 *   timeSource = "dhcp"    take the server from DHCP option 42
 *   timeSource = "static"  poll Settings::ntpServer()
 *   timeSource = "off"     leave the clock alone
 *
 * Two ordering details decide the design:
 *
 *  - lwIP only accepts option 42 if sntp_servermode_dhcp() is enabled *before*
 *    the DHCP ACK is processed, so begin() has to run before esp_eth_start().
 *  - the lease (and with it the option-42 address) arrives as
 *    IP_EVENT_ETH_GOT_IP, so onGotIp() re-polls immediately instead of waiting
 *    out the SNTP interval.
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

#include "settings.h"

/* Local ISO-8601 timestamp ("2026-11-11T13:43:22+1100"), the format /healthz,
 * POST /ntp/sync and the sync log line all use. */
std::string isoTimestamp(time_t when);

class NtpTime {
public:
    static NtpTime &instance();

    /* Arm the client. Call once at boot, ahead of the first DHCP DISCOVER (see
     * the note above about sntp_servermode_dhcp()). */
    esp_err_t begin();

    /* Re-read Settings and restart the client if the source changed. */
    esp_err_t applySettings();

    /* A lease arrived or was renewed: poll the upstream server straight away. */
    void onGotIp();

    /* Force a poll and wait for the result: ESP_OK once the clock is set,
     * ESP_ERR_TIMEOUT if it did not happen within `timeout`, ESP_ERR_INVALID_STATE
     * when there is no active time source. */
    esp_err_t syncNow(TickType_t timeout);

    /* True once this client has set the clock from an upstream server. */
    bool synced() const { return m_last_sync.load(std::memory_order_relaxed) != 0; }

    /* Epoch seconds of the last sync, 0 if never. */
    time_t lastSync() const
    {
        return static_cast<time_t>(m_last_sync.load(std::memory_order_relaxed));
    }

    /* RFC 5905 reachability register of the current server: non-zero means it
     * has answered at least once since the last shift-out. */
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

    static void onSyncEvent(void *arg, esp_event_base_t base,
                            int32_t event_id, void *event_data);

    /* The upstream name is read on the responder task (every reply, for the
     * refid) while two other tasks rewrite it: the HTTP task on a POST /config
     * restart, the event task on a DHCP renewal. A std::string cannot be read
     * across that -- stop() clears it and frees the buffer for any name past
     * libstdc++'s 15-char SSO limit -- so it lives behind m_serverLock. The two
     * scalars are atomics instead. */
    std::string serverCopy() const;
    void        setServer(const std::string &server);

    /* lwIP's sntp_setservername() keeps the pointer it is given rather than
     * copying it, so what is handed to esp_netif_sntp_init() is this buffer and
     * not m_server: written only in start(), i.e. only after stop() has stopped
     * the client that was reading the previous contents. */
    static constexpr size_t kMaxServerName = 64;

    std::atomic<Settings::TimeSource> m_source{Settings::TimeSource::Off};
    std::atomic<bool>                 m_running{false};
    /* Taken with the singleton and never given back: this object lives as long
     * as the device. */
    mutable SemaphoreHandle_t         m_serverLock = xSemaphoreCreateMutex();
    std::string                       m_server;            /* guarded by m_serverLock */
    char                              m_lwipServer[kMaxServerName] = {};
    std::atomic<long long>            m_last_sync{0};
};
