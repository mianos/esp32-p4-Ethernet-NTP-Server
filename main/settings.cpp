#include "settings.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include "esp_log.h"
#include "nvs.h"

namespace {

const char *TAG = "settings";

constexpr const char *kNvsNamespace = "settings";
constexpr const char *kNvsKey       = "config";
/* A config blob is a handful of short strings; anything larger is junk and is
 * not worth a heap allocation to find that out. */
constexpr size_t kMaxBlobBytes = 512;

}  // namespace

Settings &Settings::instance()
{
    static Settings settings;
    return settings;
}

Settings::Settings() : m_count(0)
{
    /* Compiled-in defaults: what a fresh unit runs, and what
     * POST /config/reset restores. */
    copy(m_values.hostname, "esp32p4-ntp");
    copy(m_values.tz, "AEST-10AEDT,M10.1.0,M4.1.0/3");
    copy(m_values.timeSource, "dhcp");
    copy(m_values.ntpServer, "");

    add("hostname", m_values.hostname);
    add("tz", m_values.tz);
    add("timeSource", m_values.timeSource);
    add("ntpServer", m_values.ntpServer);
}

void Settings::copy(char *dst, const char *src)
{
    snprintf(dst, kStrLen, "%s", src);
}

void Settings::add(const char *key, char *value)
{
    if (m_count >= kMaxFields) {
        ESP_LOGE(TAG, "field table full, ignoring '%s'", key);
        return;
    }
    Field &field = m_fields[m_count++];
    field.key    = key;
    field.value  = value;
    copy(field.def, value);
}

const char *Settings::get(const char *key) const
{
    for (size_t i = 0; i < m_count; i++) {
        if (strcmp(m_fields[i].key, key) == 0) {
            return m_fields[i].value;
        }
    }
    return "";
}

bool Settings::set(const char *key, const char *value)
{
    for (size_t i = 0; i < m_count; i++) {
        if (strcmp(m_fields[i].key, key) == 0) {
            copy(m_fields[i].value, value);
            return true;
        }
    }
    return false;
}

Settings::TimeSource Settings::timeSource() const
{
    const char *mode = get("timeSource");
    if (strcmp(mode, "dhcp") == 0) {
        return TimeSource::Dhcp;
    }
    if (strcmp(mode, "static") == 0) {
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
    setenv("TZ", m_values.tz, 1);
    tzset();
}

cJSON *Settings::toJson() const
{
    cJSON *doc = cJSON_CreateObject();
    if (doc == nullptr) {
        return nullptr;
    }
    for (size_t i = 0; i < m_count; i++) {
        cJSON_AddStringToObject(doc, m_fields[i].key,
                                m_fields[i].value[0] ? m_fields[i].value : "");
    }
    return doc;
}

bool Settings::applyJson(const cJSON *doc)
{
    bool changed = false;

    for (size_t i = 0; i < m_count; i++) {
        Field       &field = m_fields[i];
        const cJSON *item  = cJSON_GetObjectItemCaseSensitive(doc, field.key);
        if (item == nullptr) {
            continue; /* POST /config accepts any subset of the keys */
        }
        if (!cJSON_IsString(item) || item->valuestring == nullptr) {
            ESP_LOGW(TAG, "'%s' ignored: expected a string", field.key);
            continue;
        }
        if (strcmp(item->valuestring, field.value) == 0) {
            continue;
        }
        if (strlen(item->valuestring) >= kStrLen) {
            ESP_LOGW(TAG, "'%s' truncated to %u characters",
                     field.key, static_cast<unsigned>(kStrLen - 1));
        }
        copy(field.value, item->valuestring);
        changed = true;
    }

    if (changed) {
        applyTimezone();
    }
    return changed;
}

void Settings::save() const
{
    cJSON *doc = toJson();
    char  *raw = doc ? cJSON_PrintUnformatted(doc) : nullptr;
    cJSON_Delete(doc);
    if (raw == nullptr) {
        ESP_LOGE(TAG, "failed to serialise config");
        return;
    }

    esp_err_t    err = ESP_FAIL;
    nvs_handle_t nvs;
    if (nvs_open(kNvsNamespace, NVS_READWRITE, &nvs) == ESP_OK) {
        err = nvs_set_str(nvs, kNvsKey, raw);
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
        nvs_close(nvs);
    }
    free(raw);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to persist config: %s", esp_err_to_name(err));
    }
}

void Settings::resetDefaults()
{
    for (size_t i = 0; i < m_count; i++) {
        copy(m_fields[i].value, m_fields[i].def);
    }
    applyTimezone();
}

void Settings::log() const
{
    char   line[256] = {0};
    size_t used      = 0;

    for (size_t i = 0; i < m_count && used < sizeof(line); i++) {
        int n = snprintf(line + used, sizeof(line) - used, "%s%s=%s",
                         used ? " " : "", m_fields[i].key,
                         m_fields[i].value[0] ? m_fields[i].value : "(unset)");
        if (n < 0) {
            break;
        }
        used += static_cast<size_t>(n);
    }
    ESP_LOGI(TAG, "%s", line);
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

    char *raw = static_cast<char *>(malloc(len));
    if (raw == nullptr) {
        nvs_close(nvs);
        ESP_LOGE(TAG, "out of memory reading config, using defaults");
        return;
    }
    err = nvs_get_str(nvs, kNvsKey, raw, &len);
    nvs_close(nvs);

    cJSON *doc = (err == ESP_OK) ? cJSON_Parse(raw) : nullptr;
    if (doc == nullptr) {
        ESP_LOGW(TAG, "stored config unparseable, using defaults");
    } else {
        applyJson(doc);
        cJSON_Delete(doc);
    }
    free(raw);
}

void Settings::begin()
{
    load();
    applyTimezone();
    log();
}
