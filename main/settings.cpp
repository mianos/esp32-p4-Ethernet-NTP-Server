#include "settings.h"

#include <cstdlib>
#include <ctime>
#include <utility>

#include "esp_log.h"
#include "nvs.h"

namespace {

constexpr const char *kNvsNamespace = "settings";
constexpr const char *kNvsKey       = "config";

/* A config blob is a handful of short strings; anything larger is junk and is
 * not worth parsing to find that out. */
constexpr size_t kMaxBlobBytes = 512;

}  // namespace

Settings &Settings::instance()
{
    static Settings settings;
    return settings;
}

Settings::Settings()
{
    /* Compiled-in defaults live in Settings::Values; registering each member
     * here snapshots it as the value POST /config/reset restores. */
    add("hostname", m_values.hostname);
    add("tz", m_values.tz);
    add("timeSource", m_values.timeSource);
    add("ntpServer", m_values.ntpServer);
}

void Settings::add(const char *key, std::string &ref)
{
    m_fields.push_back({key, &ref, ref});
}

void Settings::begin()
{
    load();
    applyTimezone();
    log();
}

void Settings::log() const
{
    std::string line;
    for (const Field &field : m_fields) {
        if (!line.empty()) {
            line += ' ';
        }
        line += field.key + '=';
        line += field.ref->empty() ? std::string{"(unset)"} : *field.ref;
    }
    ESP_LOGI(TAG, "%s", line.c_str());
}

Settings::TimeSource Settings::timeSource() const
{
    if (m_values.timeSource == "dhcp") {
        return TimeSource::Dhcp;
    }
    if (m_values.timeSource == "static") {
        return TimeSource::Static;
    }
    return TimeSource::Off;
}

const char *Settings::timeSourceName(TimeSource source)
{
    switch (source) {
    case TimeSource::Dhcp:   return "dhcp";
    case TimeSource::Static: return "static";
    case TimeSource::Off:    break;
    }
    return "off";
}

/* A POSIX TZ string, e.g. "AEST-10AEDT,M10.1.0,M4.1.0/3". Garbage simply
 * leaves localtime() on UTC, so it needs no validation of its own. */
void Settings::applyTimezone() const
{
    setenv("TZ", m_values.tz.c_str(), 1);
    tzset();
}

json::Value Settings::toJson() const
{
    json::Value doc = json::makeObject();
    if (!doc) {
        return doc;
    }
    for (const Field &field : m_fields) {
        json::addString(doc, field.key.c_str(), *field.ref);
    }
    return doc;
}

bool Settings::applyJson(const json::Value &doc)
{
    if (!doc) {
        return false;
    }

    bool changed = false;

    for (Field &field : m_fields) {
        const cJSON *item = cJSON_GetObjectItemCaseSensitive(doc.get(), field.key.c_str());
        if (item == nullptr) {
            continue;  // POST /config accepts any subset of the keys
        }
        if (!cJSON_IsString(item) || item->valuestring == nullptr) {
            ESP_LOGW(TAG, "'%s' ignored: expected a string", field.key.c_str());
            continue;
        }

        std::string value{item->valuestring};
        if (value == *field.ref) {
            continue;
        }
        if (value.size() > kMaxValueLen) {
            ESP_LOGW(TAG, "'%s' truncated to %u characters", field.key.c_str(),
                     static_cast<unsigned>(kMaxValueLen));
            value.resize(kMaxValueLen);
        }
        *field.ref = std::move(value);
        changed    = true;
    }

    if (changed) {
        applyTimezone();
    }
    return changed;
}

void Settings::save() const
{
    const std::string raw = json::print(toJson());
    if (raw.empty()) {
        ESP_LOGE(TAG, "failed to serialise config");
        return;
    }

    esp_err_t    err = ESP_FAIL;
    nvs_handle_t nvs;
    if (nvs_open(kNvsNamespace, NVS_READWRITE, &nvs) == ESP_OK) {
        err = nvs_set_str(nvs, kNvsKey, raw.c_str());
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
        nvs_close(nvs);
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to persist config: %s", esp_err_to_name(err));
    }
}

void Settings::resetDefaults()
{
    for (Field &field : m_fields) {
        *field.ref = field.def;
    }
    applyTimezone();
}

void Settings::load()
{
    nvs_handle_t nvs;
    if (nvs_open(kNvsNamespace, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGI(TAG, "no stored config, using defaults");
        return;
    }

    size_t    len = 0;
    esp_err_t err = nvs_get_str(nvs, kNvsKey, nullptr, &len);
    if (err != ESP_OK || len < 3 || len > kMaxBlobBytes) {
        nvs_close(nvs);
        ESP_LOGI(TAG, "no stored config, using defaults");
        return;
    }

    std::string raw(len, '\0');
    err = nvs_get_str(nvs, kNvsKey, raw.data(), &len);
    nvs_close(nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "reading stored config: %s", esp_err_to_name(err));
        return;
    }
    /* len includes the terminator nvs_get_str wrote. */
    raw.resize(len > 0 ? len - 1 : 0);

    json::Value doc = json::parse(raw);
    if (!doc) {
        ESP_LOGW(TAG, "stored config unparseable, using defaults");
        return;
    }
    applyJson(doc);
}
