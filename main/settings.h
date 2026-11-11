/*
 * Persistent device settings — the C counterpart of mianesp's SettingsBase.
 *
 * The whole config lives as one JSON blob in NVS. Fields are declared as plain
 * members of settings_t with compiled-in defaults, registered in
 * settings_init() with settings_field(), and loaded from NVS afterwards. A
 * missing or unparseable blob leaves the defaults in place.
 *
 * Everything here is a plain global: there is one device, one config, and the
 * HTTP handlers (web_server.c) run on the httpd task, so there is nothing to
 * hand around.
 */
#pragma once

#include <stdbool.h>

#include "cJSON.h"

/* Longest value a string setting can hold, including the terminator. */
#define SETTINGS_STR_LEN 64

typedef struct {
    /* netif hostname, and the name the DHCP client sends as option 12. Must
     * not collide with another device on the LAN. */
    char hostname[SETTINGS_STR_LEN];
    /* POSIX TZ string used by localtime() — this is presentation only; an NTP
     * server always serves UTC regardless of it. */
    char tz[SETTINGS_STR_LEN];
} settings_t;

extern settings_t g_settings;

/* Apply compiled-in defaults, then overlay whatever NVS holds. Call once,
 * before the netif is created (the DHCP hostname comes from here). */
void settings_init(void);

/* Full settings as a JSON object. Caller owns (cJSON_Delete) the result. */
cJSON *settings_to_json(void);

/* Apply whichever registered keys are present in `doc`; unknown keys and
 * wrong-typed values are ignored. Returns true if anything changed. */
bool settings_apply_json(const cJSON *doc);

/* Persist the current values as a single JSON blob in NVS. */
void settings_save(void);

/* Restore every field to its compiled-in default (POST /config/reset). */
void settings_reset_defaults(void);

/* One log line: "hostname=... tz=..." */
void settings_log(void);
