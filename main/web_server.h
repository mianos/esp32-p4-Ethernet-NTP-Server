/*
 * The mianesp HTTP surface, on plain C.
 *
 * Same wire contract as the components/webserver in mianos/mianesp, so scripts
 * and dashboards that talk to the other devices work here unchanged:
 *
 *   GET  /healthz        uptime, clock, build, heap, link and lease
 *   GET  /config         the settings JSON as stored
 *   POST /config         apply any subset of the keys, persist, echo the result
 *   POST /config/reset   restore compiled-in defaults
 *
 * Success is 2xx with the resulting state as the body; failure is
 * {"error": "...", "statusCode": N} with the matching HTTP status. Unknown
 * paths and wrong methods get the same JSON shape rather than httpd's built-in
 * plain-text page.
 */
#pragma once

#include "esp_err.h"

/* Start on :80, once, after the netif has an address. Idempotent. */
esp_err_t web_server_start(void);
