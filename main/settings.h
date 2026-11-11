/*
 * Persistent device settings — the C++ counterpart of mianesp's SettingsBase.
 *
 * The whole config lives as one JSON blob in NVS. Values are std::string
 * members of Settings::Values with compiled-in defaults, registered by add() in
 * the constructor and overlaid from NVS by begin(). A missing or unparseable
 * blob leaves the defaults in place.
 *
 * There is one device and one config, so the class is a singleton: the HTTP
 * handlers (web_server.cpp) and the clock (ntp_time.cpp) reach it through
 * Settings::instance().
 */
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "json.h"

class Settings {
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

    /* One log line: "hostname=... timeSource=... ntpServer=..." */
    void log() const;

    /* netif hostname, and the name the DHCP client sends as option 12. It must
     * not collide with another device on the LAN. */
    const std::string &hostname() const { return m_values.hostname; }

    /* POSIX TZ string used by localtime() — presentation only; an NTP server
     * always serves UTC regardless of it. */
    const std::string &tz() const { return m_values.tz; }

    /* The stored timeSource text: "off", "dhcp", "static", or junk left by a
     * hand-edited blob (timeSource() maps that to Off). */
    const std::string &timeSourceText() const { return m_values.timeSource; }
    TimeSource         timeSource() const;
    static const char *timeSourceName(TimeSource source);

    /* Upstream NTP server for timeSource "static"; empty when unset. A hostname
     * is fine as long as the DNS server from the lease answers. */
    const std::string &ntpServer() const { return m_values.ntpServer; }

    /* The settings as a JSON object; null only if the allocation failed. */
    json::Value toJson() const;

    /* Apply whichever registered keys are present in `doc`; unknown keys and
     * wrong-typed values are ignored. Returns true if anything changed. */
    bool applyJson(const json::Value &doc);

    /* Persist the current values as a single JSON blob in NVS. */
    void save() const;

    /* Restore every field to its compiled-in default (POST /config/reset). */
    void resetDefaults();

private:
    static constexpr const char *TAG = "settings";

    Settings();

    struct Values {
        std::string hostname   = "esp32p4-ntp";
        std::string tz         = "AEST-10AEDT,M10.1.0,M4.1.0/3";
        std::string timeSource = "dhcp";
        std::string ntpServer;  // empty: no static server
    };

    /* One registered setting: the member to write, plus its value at
     * registration time, which is what resetDefaults() restores. */
    struct Field {
        std::string  key;
        std::string *ref;
        std::string  def;
    };

    void add(const char *key, std::string &ref);
    void applyTimezone() const;
    void load();

    Values             m_values;
    std::vector<Field> m_fields;
};
