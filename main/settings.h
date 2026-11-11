/*
 * Persistent device settings — the C++ counterpart of mianesp's SettingsBase.
 *
 * The whole config lives as one JSON blob in NVS. Fields are declared in
 * Settings::Values with compiled-in defaults, registered by add() in the
 * constructor, and overlaid from NVS by begin(). A missing or unparseable blob
 * leaves the defaults in place.
 *
 * There is one device and one config, so the class is a singleton: the HTTP
 * handlers (web_server.cpp) and the clock (ntp_time.cpp) reach it through
 * Settings::instance().
 */
#pragma once

#include <cstddef>

#include "cJSON.h"

class Settings {
public:
    /* Longest value a string setting can hold, including the terminator. */
    static constexpr size_t kStrLen = 64;

    /* Where the system clock gets its time from. Test scaffolding: the real
     * disciplining path (GPS/PPS, or the responder trusting its own reference)
     * replaces it later. */
    enum class TimeSource {
        Off,    /* leave the clock alone: it runs from reset */
        Dhcp,   /* ask the DHCP server for option 42 */
        Static, /* poll ntpServer() */
    };

    static Settings &instance();

    /* Apply compiled-in defaults, then overlay whatever NVS holds. Call once,
     * before the netif is created (the DHCP hostname comes from here). */
    void begin();

    /* One log line: "hostname=... timeSource=... ntpServer=..." */
    void log() const;

    /* Raw access by key. get() returns "" for unknown keys; set() returns
     * false for unknown keys. */
    const char *get(const char *key) const;
    bool        set(const char *key, const char *value);

    /* Typed accessors for the fields the rest of the firmware uses. */
    const char  *hostname() const { return get("hostname"); }
    const char  *tz() const { return get("tz"); }
    TimeSource   timeSource() const;
    const char  *ntpServer() const { return get("ntpServer"); }
    static const char *timeSourceName(TimeSource source);

    /* Full settings as a JSON object. Caller owns (cJSON_Delete) the result. */
    cJSON *toJson() const;

    /* Apply whichever registered keys are present in `doc`; unknown keys and
     * wrong-typed values are ignored. Returns true if anything changed. */
    bool applyJson(const cJSON *doc);

    /* Persist the current values as a single JSON blob in NVS. */
    void save() const;

    /* Restore every field to its compiled-in default (POST /config/reset). */
    void resetDefaults();

private:
    static constexpr size_t kMaxFields = 8;

    Settings();

    struct Values {
        /* netif hostname, and the name the DHCP client sends as option 12.
         * Must not collide with another device on the LAN. */
        char hostname[kStrLen];
        /* POSIX TZ string used by localtime() — presentation only; an NTP
         * server always serves UTC regardless of it. */
        char tz[kStrLen];
        /* "off" | "dhcp" | "static" — see TimeSource. */
        char timeSource[kStrLen];
        /* Upstream NTP server, used when timeSource is "static". A hostname is
         * fine as long as the DNS server from the lease answers. */
        char ntpServer[kStrLen];
    };

    /* One registered setting: the member to write, plus its value at
     * registration time, which is what resetDefaults() restores. */
    struct Field {
        const char *key;
        char       *value;
        char        def[kStrLen];
    };

    /* Registers `value` under `key`, snapshotting its current contents as the
     * default. */
    void add(const char *key, char *value);
    void applyTimezone() const;
    void load();
    static void copy(char *dst, const char *src);

    Values m_values;
    Field  m_fields[kMaxFields];
    size_t m_count;
};

