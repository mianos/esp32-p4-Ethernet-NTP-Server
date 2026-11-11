/*
 * Device settings — a SettingsBase subclass (mianesp's settingsbase component).
 *
 * The whole config lives as one JSON blob in NVS, under the same namespace and
 * key the earlier hand-written version used ("settings"/"config"), so a device
 * carrying a stored config keeps it across this change. Values are std::string
 * members with compiled-in defaults: registering them with field() in the
 * constructor snapshots the default that resetToDefaults() restores, and
 * SettingsBase supplies load()/save()/toJson()/log().
 *
 * There is one device and one config, so instance() hands out the singleton:
 * the HTTP handlers (web_server.cpp) and the clock (ntp_time.cpp) reach it
 * through Settings::instance().
 */
#pragma once

#include <cstddef>
#include <string>

#include "JsonWrapper.h"
#include "SettingsBase.h"

class Settings : public SettingsBase {
public:
    /* Longest a string setting may hold. POST /config truncates anything
     * longer and logs it, rather than growing NVS without bound. */
    static constexpr size_t kMaxValueLen = 63;

    /* Where the system clock gets its time from. Test scaffolding: the real
     * disciplining path (GPS/PPS, or the responder trusting its own reference)
     * replaces it later. */
    enum class TimeSource {
        Off,    // leave the clock alone: it runs from reset
        Dhcp,   // ask the DHCP server for option 42
        Static, // poll ntpServer()
    };

    static Settings &instance();

    /* Overlay whatever NVS holds onto the defaults. Call once, before the netif
     * is created (the DHCP hostname comes from here). */
    void begin();

    /* netif hostname, and the name the DHCP client sends as option 12. It must
     * not collide with another device on the LAN. */
    const std::string &hostname() const { return m_hostname; }

    /* POSIX TZ string used by localtime() — presentation only; an NTP server
     * always serves UTC regardless of it. */
    const std::string &tz() const { return m_tz; }

    /* The stored timeSource text: "off", "dhcp", "static", or junk left by a
     * hand-edited blob (timeSource() maps that to Off). */
    const std::string &timeSourceText() const { return m_timeSource; }
    TimeSource         timeSource() const;
    static const char *timeSourceName(TimeSource source);

    /* Upstream NTP server for timeSource "static"; empty when unset. A hostname
     * is fine as long as the DNS server from the lease answers. */
    const std::string &ntpServer() const { return m_ntpServer; }

    /* Apply whichever registered keys are present in `doc`; unknown keys and
     * wrong-typed values are ignored. Long values are truncated to
     * kMaxValueLen. SettingsBase fires the onChange callbacks for the fields
     * that really changed. */
    void applyJson(const JsonWrapper &doc);

private:
    static constexpr const char *TAG = "settings";

    Settings(NvsStorageManager &nvs);

    /* setenv()/tzset() from the stored POSIX TZ string. Garbage there simply
     * leaves localtime() on UTC, so it needs no validation of its own. */
    void applyTimezone() const;

    /* Compiled-in defaults. The keys match the JSON field names. */
    std::string m_hostname   = "esp32p4-ntp";
    std::string m_tz         = "AEST-10AEDT,M10.1.0,M4.1.0/3";
    std::string m_timeSource = "dhcp";
    std::string m_ntpServer;
};
