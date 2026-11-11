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

#include "esp_event.h"
#include "esp_netif_sntp.h"
#include "freertos/FreeRTOS.h"

#include "settings.h"

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
    time_t lastSync() const { return static_cast<time_t>(m_last_sync.load(std::memory_order_relaxed)); }

    /* RFC 5905 reachability register of the current server: non-zero means it
     * has answered at least once since the last shift-out. */
    bool upstreamReachable() const;

    /* Current upstream server as text ("192.0.2.1" or a hostname).
     * ESP_ERR_NOT_FOUND when the client has no server yet. */
    esp_err_t upstreamText(char *buf, size_t len) const;

    const char *sourceName() const { return Settings::timeSourceName(m_source); }
    Settings::TimeSource source() const { return m_source; }

private:
    NtpTime() = default;

    esp_err_t start(Settings::TimeSource source, const char *server);
    void      stop();

    static void onSyncEvent(void *arg, esp_event_base_t base,
                            int32_t event_id, void *event_data);

    Settings::TimeSource   m_source  = Settings::TimeSource::Off;
    bool                   m_running = false;
    char                   m_server[Settings::kStrLen] = {0};

    std::atomic<long long> m_last_sync{0};
};
