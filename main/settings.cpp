#include "settings.h"

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
constexpr const char *kKeys[] = {"hostname", "tz", "timeSource", "ntpServer", "serveNtp"};

}  // namespace

Settings &Settings::instance()
{
    static NvsStorageManager nvs{kNvsNamespace};
    static Settings         settings{nvs};
    return settings;
}

Settings::Settings(NvsStorageManager &nvs)
    : SettingsBase(nvs, kNvsKey)
{
    field("hostname", m_hostname);
    field("tz", m_tz);
    field("timeSource", m_timeSource);
    field("ntpServer", m_ntpServer);
    field("serveNtp", m_serveNtp);

    /* A new TZ string takes effect immediately, including after POST
     * /config/reset. Safe to attach here: it only touches this object and the C
     * library. */
    onChange("tz", [this] { applyTimezone(); });
    onChange("timeSource", [this] {
        m_timeSourceEnum.store(parseTimeSource(m_timeSource), std::memory_order_relaxed);
    });
    onChange("serveNtp", [this] {
        m_serveNtpOn.store(m_serveNtp == "on", std::memory_order_relaxed);
    });
}

void Settings::begin()
{
    load();           // stored blob over the defaults (SettingsBase)
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
    return TimeSource::Off;
}

const char *Settings::timeSourceName(TimeSource source)
{
    switch (source) {
    case TimeSource::Dhcp:   return "dhcp";
    case TimeSource::Static: return "static";
    case TimeSource::Gps:    return "gps";
    case TimeSource::Off:    break;
    }
    return "off";
}

void Settings::applyTimezone() const
{
    setenv("TZ", m_tz.c_str(), 1);
    tzset();
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
        capped.AddItem(key, value);
    }

    loadFromJson(capped);
}
