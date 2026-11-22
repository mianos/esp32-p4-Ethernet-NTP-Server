#include "net/net_log.h"
#include "app/settings.h"

#include "net/net_addr.h"

#include <cstdlib>
#include <cstring>
#include <ctime>

#include "NvsStorageManager.h"
#include "esp_log.h"

namespace {

/* NVS namespace and key, kept from the earlier implementation. */
constexpr const char *kNvsNamespace = "settings";
constexpr const char *kNvsKey       = "config";

/* Keys POST /config accepts; others are ignored. */
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

    /* Applies at once, including after a reset. */
    onChange("tz", [this] { applyTimezone(); });
    /* Not started from onChange: settings load before lwIP exists, and socket()
     * then asserts. POST /config and the first address start it. */
    onChange("timeSource", [this] {
        m_timeSourceEnum.store(parseTimeSource(m_timeSource), std::memory_order_relaxed);
    });
    onChange("serveNtp",
             [this] { m_serveNtpOn.store(m_serveNtp == "on", std::memory_order_relaxed); });
}

void Settings::begin()
{
    load();  // stored blob over the defaults (SettingsBase)
    /* Counted and saved first, so a boot that crashes is still counted. */
    m_boots = std::to_string(std::strtoul(m_boots.c_str(), nullptr, 10) + 1);
    save();
    /* onChange only fires on a change, so seed the atomics from stored values. */
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

    /* Only the address keys are checked here. */
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
    /* Cap values so an oversized body cannot fill NVS. */
    JsonWrapper capped;
    for (const char *key : kKeys) {
        std::string value;
        if (!doc.GetField(key, value)) {
            if (doc.ContainsField(key)) {
                ESP_LOGW(TAG, "'%s' ignored: expected a string", key);
            }
            continue;
        }
        /* See kMaxHostnameLen. */
        const size_t limit = std::strcmp(key, "hostname") == 0 ? kMaxHostnameLen : kMaxValueLen;
        if (value.size() > limit) {
            ESP_LOGW(TAG, "'%s' truncated to %u characters", key, static_cast<unsigned>(limit));
            value.resize(limit);
        }
        /* Validated before it reaches NVS. Empty clears a gateway or resolver. */
        if (!validAddressing(key, value)) {
            continue;
        }
        capped.AddItem(key, value);
    }

    loadFromJson(capped);
}
