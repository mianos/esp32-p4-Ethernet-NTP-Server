#include <algorithm>
/* Device settings, a SettingsBase subclass (mianesp's settingsbase). Stored as
 * one JSON blob in NVS under "settings"/"config"; each field's default is
 * what resetToDefaults() restores. */
#pragma once

#include <atomic>
#include <cstddef>
#include <string>

#include "JsonWrapper.h"
#include "SettingsBase.h"
#include "app/text.h"

class Settings : public SettingsBase {
public:
    /* Longest string setting; POST /config truncates longer values. */
    static constexpr size_t kMaxValueLen = 63;

    /* esp_netif_set_hostname() refuses more than ESP_NETIF_HOSTNAME_MAX_SIZE, and
     * a hostname it refuses at boot cannot be fixed over the network. */
    static constexpr size_t kMaxHostnameLen = 32;

    /* "gps": the receiver and its PPS. "dhcp"/"static": the upstream SNTP client.
     * "auto": the GPS while healthy, the upstream after 15 s without a good
     * pulse (clock_source.cpp). */
    enum class TimeSource {
        Off,     // leave the clock alone: it runs from reset
        Dhcp,    // ask the DHCP server for option 42
        Static,  // poll ntpServer()
        Gps,     // u-blox over UART plus PPS
        Auto,    // GPS while it is healthy, the upstream while it is not
    };

    static Settings &instance();

    /* Load NVS over the defaults. Call before the netif exists (hostname). */
    void begin();

    /* Also DHCP option 12. */
    const std::string &hostname() const { return m_hostname; }

    /* POSIX TZ, for logs and HTTP only; NTP serves UTC. */
    const std::string &tz() const { return m_tz; }

    /* The stored text; anything unrecognised maps to Off. */
    const std::string &timeSourceText() const { return m_timeSource; }

    /* Parsed into an atomic, since the responder reads it on every reply while
     * POST /config rewrites the string on another task. */
    TimeSource timeSource() const { return m_timeSourceEnum.load(std::memory_order_relaxed); }
    static const char *timeSourceName(TimeSource source);

    /* For "static"; empty when unset. A hostname needs DNS. An IPv6 literal
     * cannot sync: lwIP's SNTP client resolves A records only. */
    const std::string &ntpServer() const { return m_ntpServer; }

    /* Serve udp/123. An unlocked device answers stratum 16, leap 3. */
    bool serveNtp() const { return m_serveNtpOn.load(std::memory_order_relaxed); }

    /* PTP mode (off, master, slave), domain and priority1. */
    const std::string &ptpMode() const { return m_ptpMode; }
    /* Clamped rather than truncated, so "300" does not become 44, and text
     * that is not a number at all is the default rather than a wrap. */
    uint8_t ptpDomain() const
    {
        const auto stored = text::toNumber<unsigned>(m_ptpDomain);
        return static_cast<uint8_t>(stored ? std::min(*stored, 127u) : 0u);
    }
    uint8_t ptpPriority1() const
    {
        const auto stored = text::toNumber<unsigned>(m_ptpPriority1);
        return static_cast<uint8_t>(stored ? std::min(*stored, 255u) : 0u);
    }

    /* The receiver's time mode, which it does not keep across a power cut:
     * "moving", "fixed" or "survey"; the held position as "x,y,z,acc[,validated]";
     * the survey as "hours,acc_mm". */
    const std::string &gpsTimeMode() const { return m_gpsTimeMode; }
    const std::string &gpsFixedPos() const { return m_gpsFixedPos; }
    const std::string &gpsSurvey() const { return m_gpsSurvey; }
    /* Antenna cable delay in ns (about 5 ns per metre of coax). */
    const std::string &antDelayNs() const { return m_antDelayNs; }
    void               setAntDelayNs(const std::string &v) { m_antDelayNs = v; }
    /* User delay in ns: a measured pulse offset to correct. */
    const std::string &gpsUserDelayNs() const { return m_gpsUserDelayNs; }
    void               setGpsUserDelayNs(const std::string &v) { m_gpsUserDelayNs = v; }
    /* Log copy target: dotted-quad host and port; empty host is off. */
    const std::string &logHost() const { return m_logHost; }
    const std::string &logPort() const { return m_logPort; }
    const std::string &boots() const { return m_boots; }
    /* Kept by a config reset; it describes the device, not the config. */
    void setBoots(const std::string &v) { m_boots = v; }

    /* Last working UART rate, so a boot starts there. Written by GpsTime. */
    const std::string &gpsBaud() const { return m_gpsBaud; }
    void               setGpsBaud(const std::string &v) { m_gpsBaud = v; }

    /* Addressing: ipMode "dhcp" or "static", the address as "a.b.c.d/len".
     * Checked when written (settings.cpp) and again when applied (eth_link.cpp,
     * which falls back to DHCP). */
    const std::string &ipMode() const { return m_ipMode; }
    const std::string &ipAddress() const { return m_ipAddress; }
    const std::string &ipGateway() const { return m_ipGateway; }
    const std::string &ipDns() const { return m_ipDns; }
    void               setIpMode(const std::string &v) { m_ipMode = v; }
    void               setIpAddress(const std::string &v) { m_ipAddress = v; }
    void               setIpGateway(const std::string &v) { m_ipGateway = v; }
    void               setIpDns(const std::string &v) { m_ipDns = v; }
    void               setGpsTimeMode(const std::string &v) { m_gpsTimeMode = v; }
    void               setGpsFixedPos(const std::string &v) { m_gpsFixedPos = v; }
    void               setGpsSurvey(const std::string &v) { m_gpsSurvey = v; }

    /* Apply the registered keys present in `doc`, truncating long values;
     * unknown keys and wrong types are ignored. onChange fires for real changes. */
    void applyJson(const JsonWrapper &doc);

private:
    static constexpr const char *TAG = "settings";

    Settings(NvsStorageManager &nvs);

    /* Validation for the addressing keys; false keeps the stored value. */
    static bool validAddressing(const char *key, const std::string &value);

    /* setenv()/tzset(); an invalid string leaves localtime() on UTC. */
    void applyTimezone() const;

    /* Compiled-in defaults; names match the JSON keys. */
    std::string m_hostname   = "esp32p4-ntp";
    std::string m_tz         = "AEST-10AEDT,M10.1.0,M4.1.0/3";
    std::string m_timeSource = "dhcp";
    std::string m_ntpServer;
    std::string m_serveNtp     = "on";
    std::string m_ptpMode      = "off";
    std::string m_ptpDomain    = "0";
    std::string m_ptpPriority1 = "128";
    std::string m_gpsTimeMode  = "moving";
    std::string m_gpsFixedPos;
    std::string m_gpsSurvey;
    std::string m_antDelayNs     = "0";
    std::string m_gpsUserDelayNs = "0";
    std::string m_logHost;
    std::string m_logPort = "5514";
    std::string m_boots   = "0";
    std::string m_gpsBaud = "38400";
    std::string m_ipMode  = "dhcp";
    std::string m_ipAddress;
    std::string m_ipGateway;
    std::string m_ipDns;

    /* "off", "dhcp", "static", "gps", "auto"; anything else is Off. */
    static TimeSource parseTimeSource(const std::string &text);

    /* See timeSource(). */
    std::atomic<TimeSource> m_timeSourceEnum{TimeSource::Dhcp};

    /* serveNtp() is read from other tasks, so it is kept parsed here. */
    std::atomic<bool> m_serveNtpOn{true};
};
