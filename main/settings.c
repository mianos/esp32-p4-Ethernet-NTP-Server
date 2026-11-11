#include "settings.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "settings";

#define NVS_NAMESPACE  "settings"
#define NVS_KEY        "config"
#define MAX_FIELDS     8
/* A config blob is a handful of short strings; anything larger is junk and is
 * not worth a heap allocation to find that out. */
#define MAX_BLOB_BYTES 512

settings_t g_settings;

/* One registered setting: the member to write, plus its value at registration
 * time, which is what resetToDefaults()/POST /config/reset restores. */
typedef struct {
    const char *key;
    char       *ref;
    char        def[SETTINGS_STR_LEN];
} field_t;

static field_t s_fields[MAX_FIELDS];
static int     s_field_count;

static void copy_str(char *dst, const char *src)
{
    snprintf(dst, SETTINGS_STR_LEN, "%s", src);
}

static void field(const char *key, char *ref)
{
    if (s_field_count >= MAX_FIELDS) {
        ESP_LOGE(TAG, "field table full, ignoring '%s'", key);
        return;
    }
    field_t *f = &s_fields[s_field_count++];
    f->key = key;
    f->ref = ref;
    copy_str(f->def, ref);
}

/* A POSIX TZ string, e.g. "AEST-10AEDT,M10.1.0,M4.1.0/3". Garbage simply
 * leaves localtime() on UTC, so it needs no validation of its own. */
static void apply_timezone(void)
{
    setenv("TZ", g_settings.tz, 1);
    tzset();
}

cJSON *settings_to_json(void)
{
    cJSON *doc = cJSON_CreateObject();
    if (doc == NULL) {
        return NULL;
    }
    for (int i = 0; i < s_field_count; i++) {
        cJSON_AddStringToObject(doc, s_fields[i].key,
                                s_fields[i].ref[0] ? s_fields[i].ref : "");
    }
    return doc;
}

bool settings_apply_json(const cJSON *doc)
{
    bool changed = false;

    for (int i = 0; i < s_field_count; i++) {
        const field_t *f = &s_fields[i];
        const cJSON   *item = cJSON_GetObjectItemCaseSensitive(doc, f->key);
        if (item == NULL) {
            continue; /* POST /config accepts any subset of the keys */
        }
        if (!cJSON_IsString(item) || item->valuestring == NULL) {
            ESP_LOGW(TAG, "'%s' ignored: expected a string", f->key);
            continue;
        }
        if (strcmp(item->valuestring, f->ref) == 0) {
            continue;
        }
        if (strlen(item->valuestring) >= SETTINGS_STR_LEN) {
            ESP_LOGW(TAG, "'%s' truncated to %d characters",
                     f->key, SETTINGS_STR_LEN - 1);
        }
        copy_str(f->ref, item->valuestring);
        changed = true;
    }

    if (changed) {
        apply_timezone();
    }
    return changed;
}

void settings_save(void)
{
    cJSON *doc = settings_to_json();
    char  *raw = doc ? cJSON_PrintUnformatted(doc) : NULL;
    cJSON_Delete(doc);
    if (raw == NULL) {
        ESP_LOGE(TAG, "failed to serialise config");
        return;
    }

    esp_err_t    err = ESP_FAIL;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        err = nvs_set_str(nvs, NVS_KEY, raw);
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

void settings_reset_defaults(void)
{
    for (int i = 0; i < s_field_count; i++) {
        copy_str(s_fields[i].ref, s_fields[i].def);
    }
    apply_timezone();
}

void settings_log(void)
{
    char   line[256] = {0};
    size_t used = 0;

    for (int i = 0; i < s_field_count && used < sizeof(line); i++) {
        int n = snprintf(line + used, sizeof(line) - used, "%s%s=%s",
                         used ? " " : "", s_fields[i].key,
                         s_fields[i].ref[0] ? s_fields[i].ref : "(unset)");
        if (n < 0) {
            break;
        }
        used += (size_t)n;
    }
    ESP_LOGI(TAG, "%s", line);
}

static void load(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGI(TAG, "no stored config, using defaults");
        return;
    }

    size_t    len = 0;
    esp_err_t err = nvs_get_str(nvs, NVS_KEY, NULL, &len);
    if (err != ESP_OK || len < 3 || len > MAX_BLOB_BYTES) {
        nvs_close(nvs);
        ESP_LOGI(TAG, "no stored config, using defaults");
        return;
    }

    char *raw = malloc(len);
    if (raw == NULL) {
        nvs_close(nvs);
        ESP_LOGE(TAG, "out of memory reading config, using defaults");
        return;
    }
    err = nvs_get_str(nvs, NVS_KEY, raw, &len);
    nvs_close(nvs);

    cJSON *doc = (err == ESP_OK) ? cJSON_Parse(raw) : NULL;
    if (doc == NULL) {
        ESP_LOGW(TAG, "stored config unparseable, using defaults");
    } else {
        settings_apply_json(doc);
        cJSON_Delete(doc);
    }
    free(raw);
}

void settings_init(void)
{
    /* Compiled-in defaults: what a fresh unit runs, and what
     * POST /config/reset restores. hostname must not collide with another
     * device on the LAN, so it also shows up in /healthz and the DHCP lease. */
    copy_str(g_settings.hostname, "esp32p4-ntp");
    copy_str(g_settings.tz, "AEST-10AEDT,M10.1.0,M4.1.0/3");

    field("hostname", g_settings.hostname);
    field("tz",       g_settings.tz);

    load();
    apply_timezone();
    settings_log();
}
