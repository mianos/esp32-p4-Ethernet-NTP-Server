#include "net/net_log.h"
#include "app/settings.h"

#include "net/net_addr.h"

#include <cstdlib>
#include <cstring>
#include <ctime>

#include "NvsStorageManager.h"
#include "esp_log.h"

namespace {

/* The same namespace/key the hand-written implementation used, so an existing
 * stored blob is still read. */
constexpr const char *kNvsNamespace = "settings";
constexpr const char *kNvsKey       = "config";

/* The keys POST /config accepts. Anything else in the body is ignored, as it
 * always was. */
/* Writable through POST /config. gpsFixedPos is here because a known antenna
 * position should be provisioned rather than surveyed: it is the one input that
 * lets the receiver stop solving for itself, which is what the interface
 * description asks for before any timing claim. */
constexpr const char *kKeys[] = {"hostname",    "tz",          "timeSource", "ntpServer",
                                 "serveNtp",    "gpsFixedPos", "logHost",    "logPort",
                                 "ipMode",      "ipAddress",   "ipGateway",  "ipDns",
                                 "gpsTimeMode", "gpsSurvey",   "antDelayNs", "gpsUserDelayNs",
                                 "gpsBaud",     "ptpMode",     "ptpDomain",  "ptpPriority1"};

}  // namespace

Settings &Settings::instance()
{
    static NvsStorageManager nvs{kNvsNamespace};
    static Settings          settings{nvs};
    return settings;
}

Settings::Settings(NvsStorageManager &nvs) : SettingsBase(nvs, kNvsKey)
{
    field("hostname", m_hostname);
    field("tz", m_tz);
    field("timeSource", m_timeSource);
    field("ntpServer", m_ntpServer);
    field("serveNtp", m_serveNtp);
    field("ptpMode", m_ptpMode);
    field("ptpDomain", m_ptpDomain);
    field("ptpPriority1", m_ptpPriority1);
    field("gpsTimeMode", m_gpsTimeMode);
    field("gpsFixedPos", m_gpsFixedPos);
    field("gpsSurvey", m_gpsSurvey);
    field("antDelayNs", m_antDelayNs);
    field("gpsUserDelayNs", m_gpsUserDelayNs);
    field("logHost", m_logHost);
    field("logPort", m_logPort);
    field("boots", m_boots);
    field("gpsBaud", m_gpsBaud);
    field("ipMode", m_ipMode);
    field("ipAddress", m_ipAddress);
    field("ipGateway", m_ipGateway);
    field("ipDns", m_ipDns);

    /* A new TZ string takes effect immediately, including after POST
     * /config/reset. Safe to attach here: it only touches this object and the C
     * library. */
    onChange("tz", [this] { applyTimezone(); });
    /* The log sink is deliberately *not* started from onChange: settings load
     * happens before lwIP exists, and socket() with no tcpip thread asserts
     * "Invalid mbox" on every boot. The POST handler starts it instead, and the
     * lease starts it again at boot. */
    onChange("timeSource", [this] {
        m_timeSourceEnum.store(parseTimeSource(m_timeSource), std::memory_order_relaxed);
    });
    onChange("serveNtp",
             [this] { m_serveNtpOn.store(m_serveNtp == "on", std::memory_order_relaxed); });
}

void Settings::begin()
{
    load();  // stored blob over the defaults (SettingsBase)
    /* Counted before anything else can fail, and saved at once: the interesting
     * boot is the one that does not reach the end. */
    m_boots = std::to_string(std::strtoul(m_boots.c_str(), nullptr, 10) + 1);
    save();
    /* The onChange hooks only fire for a field that actually changed, so a
     * stored value equal to the default would never reach the atomics without
     * this. */
    m_timeSourceEnum.store(parseTimeSource(m_timeSource), std::memory_order_relaxed);
    m_serveNtpOn.store(m_serveNtp == "on", std::memory_order_relaxed);
    applyTimezone();  // before the first timestamped log line
    log();            // "hostname=... tz=... timeSource=... ntpServer=..."
}

Settings::TimeSource Settings::parseTimeSource(const std::string &text)
{
    if (text == "dhcp") {
        return TimeSource::Dhcp;
    }
    if (text == "static") {
        return TimeSource::Static;
    }
    if (text == "gps") {
        return TimeSource::Gps;
    }
    if (text == "auto") {
        return TimeSource::Auto;
    }
    return TimeSource::Off;
}

const char *Settings::timeSourceName(TimeSource source)
{
    switch (source) {
    case TimeSource::Dhcp:   return "dhcp";
    case TimeSource::Static: return "static";
    case TimeSource::Gps:    return "gps";
    case TimeSource::Auto:   return "auto";
    case TimeSource::Off:    break;
    }
    return "off";
}

void Settings::applyTimezone() const
{
    setenv("TZ", m_tz.c_str(), 1);
    tzset();
}

bool Settings::validAddressing(const char *key, const std::string &value)
{
    if (std::strcmp(key, "ipMode") == 0) {
        if (value == "dhcp" || value == "static") {
            return true;
        }
        ESP_LOGW(TAG, "ipMode '%s' ignored: expected dhcp or static", value.c_str());
        return false;
    }
    if (std::strcmp(key, "ipAddress") == 0) {
        esp_ip4_addr_t addr{};
        uint8_t        prefix = 0;
        if (value.empty() || net_addr::parseCidr(value, addr, prefix)) {
            return true;
        }
        ESP_LOGW(TAG, "ipAddress '%s' ignored: expected a.b.c.d/len with len 1..30", value.c_str());
        return false;
    }
    if (std::strcmp(key, "gpsTimeMode") == 0) {
        if (value == "moving" || value == "fixed" || value == "survey") {
            return true;
        }
        ESP_LOGW(TAG, "gpsTimeMode '%s' ignored: expected moving, fixed or survey", value.c_str());
        return false;
    }
    if (std::strcmp(key, "gpsBaud") == 0) {
        if (value.size() <= 6 && !value.empty() &&
            value.find_first_not_of("0123456789") == std::string::npos) {
            return true;
        }
        ESP_LOGW(TAG, "gpsBaud '%s' ignored: expected a number", value.c_str());
        return false;
    }

    /* Only the two quad-shaped keys are checked here; everything else in
     * kKeys validates itself or not at all. */
    if (std::strcmp(key, "ipGateway") != 0 && std::strcmp(key, "ipDns") != 0) {
        return true;
    }
    esp_ip4_addr_t addr{};
    if (value.empty() || net_addr::parseQuad(value, addr)) {
        return true;
    }
    ESP_LOGW(TAG, "%s '%s' ignored: expected a dotted quad", key, value.c_str());
    return false;
}

void Settings::applyJson(const JsonWrapper &doc)
{
    /* SettingsBase applies whatever keys it sees, so cap the values on the way
     * in: a hand-crafted 4 kB body would otherwise sit in NVS forever. */
    JsonWrapper capped;
    for (const char *key : kKeys) {
        std::string value;
        if (!doc.GetField(key, value)) {
            if (doc.ContainsField(key)) {
                ESP_LOGW(TAG, "'%s' ignored: expected a string", key);
            }
            continue;
        }
        /* The hostname has a smaller cap than the rest: esp_netif_set_hostname()
         * refuses more than 32 bytes, and at boot that refusal is fatal, so the
         * limit has to be applied where the value is stored. */
        const size_t limit = std::strcmp(key, "hostname") == 0 ? kMaxHostnameLen : kMaxValueLen;
        if (value.size() > limit) {
            ESP_LOGW(TAG, "'%s' truncated to %u characters", key, static_cast<unsigned>(limit));
            value.resize(limit);
        }
        /* Addressing is validated here rather than in eth_link.cpp, so a bad
         * value never reaches NVS and never survives a reboot. An empty string
         * is a legitimate way to clear a gateway or a resolver. */
        if (!validAddressing(key, value)) {
            continue;
        }
        capped.AddItem(key, value);
    }

    loadFromJson(capped);
}
