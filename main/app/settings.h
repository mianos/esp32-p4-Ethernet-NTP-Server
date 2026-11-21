#include <algorithm>
#include <cstdlib>
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

#include <atomic>
#include <cstddef>
#include <string>

#include "JsonWrapper.h"
#include "SettingsBase.h"

class Settings : public SettingsBase {
public:
    /* Longest a string setting may hold. POST /config truncates anything
     * longer and logs it, rather than growing NVS without bound. */
    static constexpr size_t kMaxValueLen = 63;

    /* ...except the hostname, which has its own, smaller cap: it is handed
     * straight to esp_netif_set_hostname(), and that refuses anything over
     * ESP_NETIF_HOSTNAME_MAX_SIZE (32). The refusal is not cosmetic: at boot
     * there is no one to tell and no second interface to recover over, so a
     * value of that size must never reach NVS in the first place. */
    static constexpr size_t kMaxHostnameLen = 32;

    /* Where the system clock gets its time from. "gps" is the stratum-1 path:
     * the u-blox receiver on UART1, disciplined by its PPS line (gps_time.cpp).
     * The others are the upstream SNTP client.
     *
     * "auto" is gps with an upstream behind it: the receiver is served and
     * steered while it is healthy, and the moment it stops correcting (no
     * pulse for fifteen seconds) the SNTP client is started and takes the
     * clock, the served reference and the stratum with it. The rule that makes
     * it safe is that exactly one of them owns CLOCK_REALTIME at any instant;
     * see clock_source.cpp. "gps" stays strict, so a device configured before
     * this existed keeps the behaviour it was configured for. */
    enum class TimeSource {
        Off,     // leave the clock alone: it runs from reset
        Dhcp,    // ask the DHCP server for option 42
        Static,  // poll ntpServer()
        Gps,     // u-blox over UART plus PPS
        Auto,    // GPS while it is healthy, the upstream while it is not
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

    /* The stored timeSource text: "off", "dhcp", "static", "gps", "auto", or
     * junk left by a hand-edited blob (timeSource() maps that to Off). */
    const std::string &timeSourceText() const { return m_timeSource; }

    /* The parsed answer, not the string: the responder task asks this on every
     * reply (to pick the TimeRef) while POST /config rewrites the string above
     * on the HTTP task. Reading a std::string across that is a data race -- and
     * a 20-character junk value would be a heap buffer being freed under the
     * reader -- so the enum lives in an atomic, seeded in begin() and refreshed
     * by the onChange hook, exactly like serveNtp() below. */
    TimeSource timeSource() const { return m_timeSourceEnum.load(std::memory_order_relaxed); }
    static const char *timeSourceName(TimeSource source);

    /* Upstream NTP server for timeSource "static"; empty when unset. A hostname
     * is fine as long as the DNS server from the lease answers. An IPv6 literal
     * is stored but cannot sync: lwIP's SNTP client resolves A records only
     * (ntp_time.cpp logs a warning when it sees one). */
    const std::string &ntpServer() const { return m_ntpServer; }

    /* Serve NTP on udp/123 ("on"/"off"). Serving is honest by construction: a
     * device that is not locked answers with stratum 16 and leap indicator 3,
     * which tells every client to ignore it, so leaving this on cannot mislead
     * anything -- it can only spend electricity. */
    bool serveNtp() const { return m_serveNtpOn.load(std::memory_order_relaxed); }

    /* PTP: off, master or slave; the domain and the priority1 the Announce
     * carries. Read by the responder at start and by the console. */
    const std::string &ptpMode() const { return m_ptpMode; }
    /* Clamped, not cast: "300" through a uint8_t cast is 44, which is a
     * different domain rather than a refused one. */
    uint8_t ptpDomain() const
    {
        return static_cast<uint8_t>(
            std::min(std::strtoul(m_ptpDomain.c_str(), nullptr, 10), 127ul));
    }
    uint8_t ptpPriority1() const
    {
        return static_cast<uint8_t>(
            std::min(std::strtoul(m_ptpPriority1.c_str(), nullptr, 10), 255ul));
    }

    /* Where the receiver's time mode was left, so a power cut does not cost it.
     * The grid goes off, and a locked position or a survey in progress lives in
     * the module's RAM and dies with the supply -- there is no other copy.
     *
     * Three strings, encoded rather than structured, because that is what this
     * config holds: "moving", "fixed" or "survey"; the fixed position as
     * "x,y,z,acc"; and the survey parameters as "hours,acc_mm" so a wanted
     * survey can be resumed rather than silently forgotten. */
    const std::string &gpsTimeMode() const { return m_gpsTimeMode; }
    const std::string &gpsFixedPos() const { return m_gpsFixedPos; }
    const std::string &gpsSurvey() const { return m_gpsSurvey; }
    /* The antenna lead's one-way delay, in nanoseconds, for the receiver to
     * subtract. A constant that never averages out, so it is the one error a
     * single receiver cannot measure for itself and the installer has to state.
     * Coax runs about 5 ns/m whatever its loss class, so the number is ~5x the
     * cable length. */
    const std::string &antDelayNs() const { return m_antDelayNs; }
    void               setAntDelayNs(const std::string &v) { m_antDelayNs = v; }
    /* The calibration term: what the pulse was measured to be out, entered as a
     * delay so the receiver moves the edge by the same amount. Nothing inside
     * the receiver can see a constant error, which is why it has to be measured
     * outside and written in. */
    const std::string &gpsUserDelayNs() const { return m_gpsUserDelayNs; }
    void               setGpsUserDelayNs(const std::string &v) { m_gpsUserDelayNs = v; }
    /* Power cuts are a normal event here -- street works, storms -- so the count
     * of boots is worth carrying: a survey that vanished is otherwise
     * indistinguishable from one that was never started. */
    /* Where to duplicate the console to, as a dotted quad and a port. Empty
     * host means no sink; the two are kept as strings for the same reason the
     * rest of this config is. */
    const std::string &logHost() const { return m_logHost; }
    const std::string &logPort() const { return m_logPort; }
    const std::string &boots() const { return m_boots; }
    /* Settable so a config reset can put the count back: the counter is the one
     * thing in the blob that records the device rather than the config, and
     * losing it hides the reboot that a reset usually precedes. */
    void setBoots(const std::string &v) { m_boots = v; }

    /* How the interface is addressed. "dhcp" is what the box has always done
     * and stays the default; "static" exists for a link with no DHCP server on
     * it, where the alternative is not having the link at all.
     *
     * These are the only settings that can strand a device, so they are
     * validated where they are written -- settings.cpp rejects what does not
     * parse, eth_link.cpp falls back to DHCP if it still somehow does not --
     * and the console is the way back in.
     *
     * The address is stored as "a.b.c.d/len": one string, one parser, and no
     * way to save a netmask that disagrees with the prefix. */
    /* The UART rate the receiver was last heard on. The module keeps its own
     * rate across a power cut, so this is what lets a boot go straight to it
     * instead of walking the scan while the receiver is still starting; the
     * scan stays as the fallback for a module that has been reset to something
     * else. Written by GpsTime, read once in its begin(). */
    const std::string &gpsBaud() const { return m_gpsBaud; }
    void               setGpsBaud(const std::string &v) { m_gpsBaud = v; }

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

    /* Apply whichever registered keys are present in `doc`; unknown keys and
     * wrong-typed values are ignored. Long values are truncated to
     * kMaxValueLen. SettingsBase fires the onChange callbacks for the fields
     * that really changed. */
    void applyJson(const JsonWrapper &doc);

private:
    static constexpr const char *TAG = "settings";

    Settings(NvsStorageManager &nvs);

    /* Per-key validation for the addressing settings; false means the value in
     * the request is dropped and the stored one is kept. */
    static bool validAddressing(const char *key, const std::string &value);

    /* setenv()/tzset() from the stored POSIX TZ string. Garbage there simply
     * leaves localtime() on UTC, so it needs no validation of its own. */
    void applyTimezone() const;

    /* Compiled-in defaults. The keys match the JSON field names. */
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

    /* "off", "dhcp", "static", "gps": anything else is Off. Used to seed
     * m_timeSourceEnum in begin() and on every change. */
    static TimeSource parseTimeSource(const std::string &text);

    /* See timeSource(). Seeded in begin() and refreshed by the constructor's
     * onChange hook, for the same reason m_serveNtpOn exists: the reader and the
     * writer are on different tasks. */
    std::atomic<TimeSource> m_timeSourceEnum{TimeSource::Dhcp};

    /* The parsed answer, because the two sides are on different tasks: the
     * responder asks serveNtp() from its Ethernet event handler, while POST
     * /config rewrites the string above on the HTTP task. Keeping the answer in
     * an atomic means that read never touches the string. Seeded in begin()
     * and refreshed by the onChange hook the constructor installs. */
    std::atomic<bool> m_serveNtpOn{true};
};
