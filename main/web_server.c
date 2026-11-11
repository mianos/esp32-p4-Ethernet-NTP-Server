#include "web_server.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>

#include "arpa/inet.h"
#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

#include "net_state.h"
#include "settings.h"

static const char *TAG = "webserver";

/* /config bodies are tiny — bound the input so a hostile Content-Length cannot
 * turn into a multi-kilobyte allocation. */
#define MAX_JSON_BODY_BYTES 4096

static httpd_handle_t s_server;

/* -------------------------------------------------------------------------
 * JSON replies
 * ---------------------------------------------------------------------- */

/* The status line needs a reason phrase; a bare number is malformed HTTP. */
static const char *status_line(int status_code)
{
    switch (status_code) {
    case 200: return "200 OK";
    case 202: return "202 Accepted";
    case 400: return "400 Bad Request";
    case 404: return "404 Not Found";
    case 405: return "405 Method Not Allowed";
    case 409: return "409 Conflict";
    case 413: return "413 Payload Too Large";
    case 422: return "422 Unprocessable Entity";
    case 503: return "503 Service Unavailable";
    case 504: return "504 Gateway Timeout";
    default:  return "500 Internal Server Error";
    }
}

/* Sends `json` as the body and takes ownership of it. */
static esp_err_t send_json(httpd_req_t *req, cJSON *json, int status_code)
{
    if (json == NULL) {
        return httpd_resp_send_500(req);
    }
    char *out = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    if (out == NULL) {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_status(req, status_line(status_code));
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_sendstr(req, out);
    free(out);
    return err;
}

static esp_err_t send_json_error(httpd_req_t *req, int status_code, const char *message)
{
    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        return httpd_resp_send_500(req);
    }
    cJSON_AddStringToObject(json, "error", message);
    cJSON_AddNumberToObject(json, "statusCode", status_code);
    return send_json(req, json, status_code);
}

static esp_err_t not_found_handler(httpd_req_t *req, httpd_err_code_t err)
{
    /* req->uri is up to 512 bytes (CONFIG_HTTPD_MAX_URI_LEN), so the message
     * buffer has to hold the whole path plus the prefix. */
    char message[600];
    snprintf(message, sizeof(message), "no such endpoint: %s", req->uri);
    return send_json_error(req, 404, message);
}

static esp_err_t method_not_allowed_handler(httpd_req_t *req, httpd_err_code_t err)
{
    char message[600];
    snprintf(message, sizeof(message), "method not allowed on %s", req->uri);
    return send_json_error(req, 405, message);
}

/* Peer address of the request, for logging who changed what. */
static void caller_address(httpd_req_t *req, char *buf, size_t len)
{
    struct sockaddr_storage addr;
    socklen_t               addr_len = sizeof(addr);

    if (getpeername(httpd_req_to_sockfd(req), (struct sockaddr *)&addr, &addr_len) != 0) {
        snprintf(buf, len, "?");
        return;
    }
    if (addr.ss_family == AF_INET) {
        inet_ntop(AF_INET, &((struct sockaddr_in *)&addr)->sin_addr, buf, len);
        return;
    }

    char v6[INET6_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET6, &((struct sockaddr_in6 *)&addr)->sin6_addr, v6, sizeof(v6));
    /* The server socket is dual-stack, so IPv4 peers arrive v4-mapped. */
    if (strncasecmp(v6, "::ffff:", 7) == 0) {
        snprintf(buf, len, "%s", v6 + 7);
        return;
    }
    snprintf(buf, len, "%s", v6);
}

/* Reads exactly content_len bytes; NULL if the body arrived short. */
static char *read_body(httpd_req_t *req)
{
    char *body = malloc((size_t)req->content_len + 1);
    if (body == NULL) {
        return NULL;
    }

    int received = 0;
    while (received < req->content_len) {
        int got = httpd_req_recv(req, body + received, req->content_len - received);
        if (got == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (got <= 0) {
            free(body);
            return NULL;
        }
        received += got;
    }
    body[received] = '\0';
    return body;
}

/* -------------------------------------------------------------------------
 * /healthz
 * ---------------------------------------------------------------------- */

/* The RTC reads 1970 until something sets it, and this device has no RTC of
 * its own, so say which it is instead of presenting 1970 as a fact. */
static bool clock_is_set(void)
{
    return time(NULL) >= 1577836800; /* 2020-01-01T00:00:00Z */
}

static void local_time_string(char *buf, size_t len)
{
    time_t    now = time(NULL);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    strftime(buf, len, "%Y-%m-%dT%H:%M:%S%z", &tm_now);
}

static esp_err_t healthz_handler(httpd_req_t *req)
{
    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        return httpd_resp_send_500(req);
    }

    char timebuf[32] = {0};
    local_time_string(timebuf, sizeof(timebuf));

    cJSON_AddNumberToObject(json, "uptime", (double)(esp_timer_get_time() / 1000000ULL));
    cJSON_AddStringToObject(json, "time", timebuf);
    cJSON_AddBoolToObject(json, "time_set", clock_is_set());

    const esp_app_desc_t *desc = esp_app_get_description();
    if (desc != NULL) {
        cJSON_AddStringToObject(json, "version", desc->version);
        cJSON_AddStringToObject(json, "idf_ver", desc->idf_ver);
    }
    const esp_partition_t *running = esp_ota_get_running_partition();
    cJSON_AddStringToObject(json, "partition", running ? running->label : "?");
    cJSON_AddNumberToObject(json, "heap_free", (double)esp_get_free_heap_size());

    cJSON_AddStringToObject(json, "hostname", g_settings.hostname);
    cJSON_AddBoolToObject(json, "link_up", net_state_link_up());

    esp_netif_ip_info_t ip_info;
    esp_netif_t        *netif = net_state_netif();
    if (netif != NULL && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK &&
        ip_info.ip.addr != 0) {
        char ipbuf[16];
        snprintf(ipbuf, sizeof(ipbuf), IPSTR, IP2STR(&ip_info.ip));
        cJSON_AddStringToObject(json, "ip", ipbuf);
    }

    return send_json(req, json, 200);
}

/* -------------------------------------------------------------------------
 * /config
 * ---------------------------------------------------------------------- */

static esp_err_t config_get_handler(httpd_req_t *req)
{
    return send_json(req, settings_to_json(), 200);
}

/* The DHCP client embeds the hostname in its requests (option 12), but it only
 * builds them at DISCOVER/REQUEST time, so a change here reaches the lease on
 * the next renewal. */
static void apply_hostname(const char *hostname)
{
    esp_netif_t *netif = net_state_netif();
    if (netif == NULL) {
        return;
    }
    esp_err_t err = esp_netif_set_hostname(netif, hostname);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set hostname '%s': %s", hostname, esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "hostname now %s (DHCP sends it on the next lease)", hostname);
}

static esp_err_t config_post_handler(httpd_req_t *req)
{
    if (req->content_len <= 0) {
        return send_json_error(req, 400, "JSON body required");
    }
    if (req->content_len > MAX_JSON_BODY_BYTES) {
        return send_json_error(req, 413, "request body too large");
    }

    char *body = read_body(req);
    if (body == NULL) {
        return send_json_error(req, 400, "request body truncated");
    }

    cJSON *doc = cJSON_Parse(body);
    free(body);
    if (doc == NULL || !cJSON_IsObject(doc)) {
        cJSON_Delete(doc);
        return send_json_error(req, 400, "invalid JSON (expected an object)");
    }

    char caller[INET6_ADDRSTRLEN];
    caller_address(req, caller, sizeof(caller));

    char hostname_before[SETTINGS_STR_LEN];
    snprintf(hostname_before, sizeof(hostname_before), "%s", g_settings.hostname);

    settings_apply_json(doc);
    cJSON_Delete(doc);
    settings_save();
    settings_log();

    if (strcmp(g_settings.hostname, hostname_before) != 0) {
        apply_hostname(g_settings.hostname);
    }
    ESP_LOGI(TAG, "POST /config from %s", caller);

    return send_json(req, settings_to_json(), 200);
}

static esp_err_t config_reset_post_handler(httpd_req_t *req)
{
    char caller[INET6_ADDRSTRLEN];
    caller_address(req, caller, sizeof(caller));

    settings_reset_defaults();
    settings_save();
    settings_log();
    apply_hostname(g_settings.hostname);
    ESP_LOGW(TAG, "POST /config/reset from %s: back to compiled-in defaults", caller);

    cJSON *resp = cJSON_CreateObject();
    if (resp == NULL) {
        return httpd_resp_send_500(req);
    }
    cJSON_AddStringToObject(resp, "status", "reset");
    return send_json(req, resp, 200);
}

/* -------------------------------------------------------------------------
 * Startup
 * ---------------------------------------------------------------------- */

typedef struct {
    const char     *uri;
    httpd_method_t  method;
    esp_err_t     (*handler)(httpd_req_t *req);
} route_t;

esp_err_t web_server_start(void)
{
    if (s_server != NULL) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    /* Keep one wedged client from holding the only socket. */
    config.lru_purge_enable = true;
    config.max_uri_handlers = 8;

    ESP_LOGI(TAG, "starting http server on port %d", config.server_port);
    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start: %s", esp_err_to_name(err));
        s_server = NULL;
        return err;
    }

    static const route_t routes[] = {
        {"/healthz",      HTTP_GET,  healthz_handler},
        {"/config",       HTTP_GET,  config_get_handler},
        {"/config",       HTTP_POST, config_post_handler},
        {"/config/reset", HTTP_POST, config_reset_post_handler},
    };

    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        httpd_uri_t uri = {
            .uri      = routes[i].uri,
            .method   = routes[i].method,
            .handler  = routes[i].handler,
            .user_ctx = NULL,
        };
        err = httpd_register_uri_handler(s_server, &uri);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "register %s %s: %s",
                     routes[i].method == HTTP_POST ? "POST" : "GET",
                     routes[i].uri, esp_err_to_name(err));
            return err;
        }
    }

    /* The built-in replies are plain text; keep every error in the JSON shape. */
    httpd_register_err_handler(s_server, HTTPD_404_NOT_FOUND, not_found_handler);
    httpd_register_err_handler(s_server, HTTPD_405_METHOD_NOT_ALLOWED, method_not_allowed_handler);

    return ESP_OK;
}
