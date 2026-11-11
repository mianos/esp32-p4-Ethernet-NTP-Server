/*
 * The mianesp HTTP surface.
 *
 * Same wire contract as the components/webserver in mianos/mianesp, so scripts
 * and dashboards that talk to the other devices work here unchanged:
 *
 *   GET  /healthz        uptime, clock, build, heap, link, lease, time source
 *   GET  /config         the settings JSON as stored
 *   POST /config         apply any subset of the keys, persist, echo the result
 *   POST /config/reset   restore compiled-in defaults
 *   POST /ntp/sync       force a poll of the upstream time server (test mode)
 *
 * Success is 2xx with the resulting state as the body; failure is
 * {"error": "...", "statusCode": N} with the matching HTTP status. Unknown
 * paths and wrong methods get the same JSON shape rather than httpd's built-in
 * plain-text page.
 *
 * httpd calls the handlers as C function pointers, so they are static members
 * with the instance reachable through instance().
 */
#pragma once

#include "cJSON.h"
#include "esp_err.h"
#include "esp_http_server.h"

class WebServer {
public:
    static WebServer &instance();

    /* Start on :80, once, after the netif has an address. Idempotent. */
    esp_err_t begin();

private:
    WebServer() = default;

    esp_err_t registerRoutes();

    static esp_err_t healthz(httpd_req_t *req);
    static esp_err_t configGet(httpd_req_t *req);
    static esp_err_t configPost(httpd_req_t *req);
    static esp_err_t configResetPost(httpd_req_t *req);
    static esp_err_t syncPost(httpd_req_t *req);

    static esp_err_t notFound(httpd_req_t *req, httpd_err_code_t err);
    static esp_err_t methodNotAllowed(httpd_req_t *req, httpd_err_code_t err);

    /* Sends `json` as the body and takes ownership of it. */
    static esp_err_t sendJson(httpd_req_t *req, cJSON *json, int status_code);
    static esp_err_t sendError(httpd_req_t *req, int status_code, const char *message);

    httpd_handle_t m_server = nullptr;
};

