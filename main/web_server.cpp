#include "web_server.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include <strings.h>

#include "arpa/inet.h"
#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "lwip/sockets.h"

#include "net_state.h"
#include "ntp_time.h"
#include "settings.h"

namespace {

const char *TAG = "webserver";

/* /config bodies are tiny — bound the input so a hostile Content-Length cannot
 * turn into a multi-kilobyte allocation. */
constexpr int kMaxJsonBodyBytes = 4096;

/* POST /ntp/sync blocks the request for at most this long. */
constexpr uint32_t kSyncWaitMs = 5000;

/* The status line needs a reason phrase; a bare number is malformed HTTP. */
const char *statusLine(int status_code)
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

/* Peer address of the request, for logging who changed what. */
void callerAddress(httpd_req_t *req, char *buf, size_t len)
{
    struct sockaddr_storage addr;
    socklen_t               addr_len = sizeof(addr);

    if (getpeername(httpd_req_to_sockfd(req), reinterpret_cast<struct sockaddr *>(&addr),
                    &addr_len) != 0) {
        snprintf(buf, len, "?");
        return;
    }
    if (addr.ss_family == AF_INET) {
        inet_ntop(AF_INET, &reinterpret_cast<struct sockaddr_in *>(&addr)->sin_addr, buf, len);
        return;
    }

    char v6[INET6_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET6, &reinterpret_cast<struct sockaddr_in6 *>(&addr)->sin6_addr, v6, sizeof(v6));
    /* The server socket is dual-stack, so IPv4 peers arrive v4-mapped. */
    if (strncasecmp(v6, "::ffff:", 7) == 0) {
        snprintf(buf, len, "%s", v6 + 7);
        return;
    }
    snprintf(buf, len, "%s", v6);
}

/* Reads exactly content_len bytes; nullptr if the body arrived short. */
char *readBody(httpd_req_t *req)
{
    char *body = static_cast<char *>(malloc(static_cast<size_t>(req->content_len) + 1));
    if (body == nullptr) {
        return nullptr;
    }

    int received = 0;
    while (received < req->content_len) {
        int got = httpd_req_recv(req, body + received, req->content_len - received);
        if (got == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (got <= 0) {
            free(body);
            return nullptr;
        }
        received += got;
    }
    body[received] = '\0';
    return body;
}

/* The RTC reads 1970 until something sets it, and this device has no RTC of
 * its own, so say which it is instead of presenting 1970 as a fact. */
bool clockIsSet()
{
    return time(nullptr) >= 1577836800; /* 2020-01-01T00:00:00Z */
}

void localTimeString(time_t when, char *buf, size_t len)
{
    struct tm tm_now;
    localtime_r(&when, &tm_now);
    strftime(buf, len, "%Y-%m-%dT%H:%M:%S%z", &tm_now);
}

/* The DHCP client embeds the hostname in its requests (option 12), but it only
 * builds them at DISCOVER/REQUEST time, so a change here reaches the lease on
 * the next renewal. */
void applyHostname(const char *hostname)
{
    esp_netif_t *netif = net_state::netif();
    if (netif == nullptr) {
        return;
    }
    esp_err_t err = esp_netif_set_hostname(netif, hostname);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set hostname '%s': %s", hostname, esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "hostname now %s (DHCP sends it on the next lease)", hostname);
}

}  // namespace

WebServer &WebServer::instance()
{
    static WebServer server;
    return server;
}

esp_err_t WebServer::sendJson(httpd_req_t *req, cJSON *json, int status_code)
{
    if (json == nullptr) {
        return httpd_resp_send_500(req);
    }
    char *out = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    if (out == nullptr) {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_status(req, statusLine(status_code));
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_sendstr(req, out);
    free(out);
    return err;
}

esp_err_t WebServer::sendError(httpd_req_t *req, int status_code, const char *message)
{
    cJSON *json = cJSON_CreateObject();
    if (json == nullptr) {
        return httpd_resp_send_500(req);
    }
    cJSON_AddStringToObject(json, "error", message);
    cJSON_AddNumberToObject(json, "statusCode", status_code);
    return sendJson(req, json, status_code);
}

esp_err_t WebServer::notFound(httpd_req_t *req, httpd_err_code_t err)
{
    (void)err;
    /* req->uri is up to 512 bytes (CONFIG_HTTPD_MAX_URI_LEN), so the message
     * buffer has to hold the whole path plus the prefix. */
    char message[600];
    snprintf(message, sizeof(message), "no such endpoint: %s", req->uri);
    return sendError(req, 404, message);
}

esp_err_t WebServer::methodNotAllowed(httpd_req_t *req, httpd_err_code_t err)
{
    (void)err;
    char message[600];
    snprintf(message, sizeof(message), "method not allowed on %s", req->uri);
    return sendError(req, 405, message);
}

esp_err_t WebServer::healthz(httpd_req_t *req)
{
    cJSON *json = cJSON_CreateObject();
    if (json == nullptr) {
        return httpd_resp_send_500(req);
    }

    char timebuf[32] = {0};
    localTimeString(time(nullptr), timebuf, sizeof(timebuf));

    cJSON_AddNumberToObject(json, "uptime", static_cast<double>(esp_timer_get_time() / 1000000ULL));
    cJSON_AddStringToObject(json, "time", timebuf);
    cJSON_AddBoolToObject(json, "time_set", clockIsSet());

    const esp_app_desc_t *desc = esp_app_get_description();
    if (desc != nullptr) {
        cJSON_AddStringToObject(json, "version", desc->version);
        cJSON_AddStringToObject(json, "idf_ver", desc->idf_ver);
    }
    const esp_partition_t *running = esp_ota_get_running_partition();
    cJSON_AddStringToObject(json, "partition", running != nullptr ? running->label : "?");
    cJSON_AddNumberToObject(json, "heap_free", static_cast<double>(esp_get_free_heap_size()));

    const Settings &settings = Settings::instance();
    const NtpTime  &ntp      = NtpTime::instance();

    cJSON_AddStringToObject(json, "hostname", settings.hostname());
    cJSON_AddBoolToObject(json, "link_up", net_state::linkUp());

    esp_netif_ip_info_t ip_info;
    esp_netif_t        *netif = net_state::netif();
    if (netif != nullptr && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK &&
        ip_info.ip.addr != 0) {
        char ipbuf[16];
        snprintf(ipbuf, sizeof(ipbuf), IPSTR, IP2STR(&ip_info.ip));
        cJSON_AddStringToObject(json, "ip", ipbuf);
    }

    /* The clock: where it is disciplined from, which upstream server is in use,
     * and whether it has actually been set rather than merely looking sane. */
    cJSON_AddStringToObject(json, "ntp_source", ntp.sourceName());
    char upstream[Settings::kStrLen];
    if (ntp.upstreamText(upstream, sizeof(upstream)) == ESP_OK) {
        cJSON_AddStringToObject(json, "ntp_server", upstream);
        cJSON_AddBoolToObject(json, "ntp_reachable", ntp.upstreamReachable());
    } else {
        cJSON_AddNullToObject(json, "ntp_server");
    }
    cJSON_AddBoolToObject(json, "ntp_synced", ntp.synced());
    if (ntp.synced()) {
        cJSON_AddNumberToObject(json, "last_sync", static_cast<double>(ntp.lastSync()));
    }

    return sendJson(req, json, 200);
}

esp_err_t WebServer::configGet(httpd_req_t *req)
{
    return sendJson(req, Settings::instance().toJson(), 200);
}

esp_err_t WebServer::configPost(httpd_req_t *req)
{
    if (req->content_len <= 0) {
        return sendError(req, 400, "JSON body required");
    }
    if (req->content_len > kMaxJsonBodyBytes) {
        return sendError(req, 413, "request body too large");
    }

    char *body = readBody(req);
    if (body == nullptr) {
        return sendError(req, 400, "request body truncated");
    }

    cJSON *doc = cJSON_Parse(body);
    free(body);
    if (doc == nullptr || !cJSON_IsObject(doc)) {
        cJSON_Delete(doc);
        return sendError(req, 400, "invalid JSON (expected an object)");
    }

    char caller[INET6_ADDRSTRLEN];
    callerAddress(req, caller, sizeof(caller));

    Settings &settings = Settings::instance();
    char      hostname_before[Settings::kStrLen];
    snprintf(hostname_before, sizeof(hostname_before), "%s", settings.hostname());

    settings.applyJson(doc);
    cJSON_Delete(doc);
    settings.save();
    settings.log();

    if (strcmp(settings.hostname(), hostname_before) != 0) {
        applyHostname(settings.hostname());
    }
    /* A changed timeSource/ntpServer takes effect now, not at the next boot. */
    NtpTime::instance().applySettings();
    ESP_LOGI(TAG, "POST /config from %s", caller);

    return sendJson(req, settings.toJson(), 200);
}

esp_err_t WebServer::configResetPost(httpd_req_t *req)
{
    char caller[INET6_ADDRSTRLEN];
    callerAddress(req, caller, sizeof(caller));

    Settings &settings = Settings::instance();
    settings.resetDefaults();
    settings.save();
    settings.log();
    applyHostname(settings.hostname());
    NtpTime::instance().applySettings();
    ESP_LOGW(TAG, "POST /config/reset from %s: back to compiled-in defaults", caller);

    cJSON *resp = cJSON_CreateObject();
    if (resp == nullptr) {
        return httpd_resp_send_500(req);
    }
    cJSON_AddStringToObject(resp, "status", "reset");
    return sendJson(req, resp, 200);
}

esp_err_t WebServer::syncPost(httpd_req_t *req)
{
    char caller[INET6_ADDRSTRLEN];
    callerAddress(req, caller, sizeof(caller));

    NtpTime  &ntp = NtpTime::instance();
    esp_err_t err = ntp.syncNow(pdMS_TO_TICKS(kSyncWaitMs));
    if (err == ESP_ERR_INVALID_STATE) {
        return sendError(req, 409, "no time source: set timeSource to dhcp or static");
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "POST /ntp/sync from %s: %s", caller, esp_err_to_name(err));
        return sendError(req, 504, "no time sync from the upstream server");
    }

    cJSON *json = cJSON_CreateObject();
    if (json == nullptr) {
        return httpd_resp_send_500(req);
    }

    char timebuf[32] = {0};
    localTimeString(time(nullptr), timebuf, sizeof(timebuf));

    cJSON_AddBoolToObject(json, "synced", true);
    cJSON_AddStringToObject(json, "time", timebuf);
    char upstream[Settings::kStrLen];
    if (ntp.upstreamText(upstream, sizeof(upstream)) == ESP_OK) {
        cJSON_AddStringToObject(json, "ntp_server", upstream);
    }
    ESP_LOGI(TAG, "POST /ntp/sync from %s: %s", caller, timebuf);

    return sendJson(req, json, 200);
}

esp_err_t WebServer::registerRoutes()
{
    struct Route {
        const char    *uri;
        httpd_method_t method;
        esp_err_t    (*handler)(httpd_req_t *req);
    };

    static const Route routes[] = {
        {"/healthz",      HTTP_GET,  healthz},
        {"/config",       HTTP_GET,  configGet},
        {"/config",       HTTP_POST, configPost},
        {"/config/reset", HTTP_POST, configResetPost},
        {"/ntp/sync",     HTTP_POST, syncPost},
    };

    for (const Route &route : routes) {
        httpd_uri_t uri = {};
        uri.uri         = route.uri;
        uri.method      = route.method;
        uri.handler     = route.handler;
        uri.user_ctx    = nullptr;

        esp_err_t err = httpd_register_uri_handler(m_server, &uri);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "register %s %s: %s",
                     route.method == HTTP_POST ? "POST" : "GET", route.uri,
                     esp_err_to_name(err));
            return err;
        }
    }
    return ESP_OK;
}

esp_err_t WebServer::begin()
{
    if (m_server != nullptr) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    /* Keep one wedged client from holding the only socket. */
    config.lru_purge_enable = true;
    config.max_uri_handlers = 8;

    ESP_LOGI(TAG, "starting http server on port %d", config.server_port);
    esp_err_t err = httpd_start(&m_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start: %s", esp_err_to_name(err));
        m_server = nullptr;
        return err;
    }

    err = registerRoutes();
    if (err != ESP_OK) {
        return err;
    }

    /* The built-in replies are plain text; keep every error in the JSON shape. */
    httpd_register_err_handler(m_server, HTTPD_404_NOT_FOUND, notFound);
    httpd_register_err_handler(m_server, HTTPD_405_METHOD_NOT_ALLOWED, methodNotAllowed);

    return ESP_OK;
}
