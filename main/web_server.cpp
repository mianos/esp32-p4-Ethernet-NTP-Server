#include "web_server.h"

#include <cstdio>
#include <cstring>
#include <ctime>

#include <optional>
#include <string>

#include "arpa/inet.h"
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
std::string callerAddress(httpd_req_t *req)
{
    struct sockaddr_storage addr;
    socklen_t               addr_len = sizeof(addr);

    if (getpeername(httpd_req_to_sockfd(req), reinterpret_cast<struct sockaddr *>(&addr),
                    &addr_len) != 0) {
        return "?";
    }

    char text[INET6_ADDRSTRLEN] = {};
    if (addr.ss_family == AF_INET) {
        inet_ntop(AF_INET, &reinterpret_cast<struct sockaddr_in *>(&addr)->sin_addr,
                  text, sizeof(text));
        return text;
    }

    inet_ntop(AF_INET6, &reinterpret_cast<struct sockaddr_in6 *>(&addr)->sin6_addr,
              text, sizeof(text));
    /* The server socket is dual-stack, so IPv4 peers arrive v4-mapped: newlib's
     * inet_ntop() writes that prefix as "::FFFF:", hence the case-insensitive
     * compare rather than std::string::starts_with(). */
    std::string peer{text};
    if (strncasecmp(peer.c_str(), "::ffff:", 7) == 0) {
        peer.erase(0, 7);
    }
    return peer;
}

/* Reads exactly content_len bytes; nullopt if the body arrived short. */
std::optional<std::string> readBody(httpd_req_t *req)
{
    std::string body(static_cast<size_t>(req->content_len), '\0');

    size_t received = 0;
    while (received < body.size()) {
        int got = httpd_req_recv(req, body.data() + received, body.size() - received);
        if (got == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (got <= 0) {
            return std::nullopt;
        }
        received += static_cast<size_t>(got);
    }
    return body;
}

/* The RTC reads 1970 until something sets it, and this device has no RTC of
 * its own, so say which it is instead of presenting 1970 as a fact. */
bool clockIsSet()
{
    return time(nullptr) >= 1577836800; /* 2020-01-01T00:00:00Z */
}

}  // namespace

WebServer &WebServer::instance()
{
    static WebServer server;
    return server;
}

/* The DHCP client embeds the hostname in its requests (option 12), but it only
 * builds them at DISCOVER/REQUEST time, so a change here reaches the lease on
 * the next renewal. */
void WebServer::applyHostname(const std::string &hostname)
{
    esp_netif_t *netif = net_state::netif();
    if (netif == nullptr) {
        return;
    }

    esp_err_t err = esp_netif_set_hostname(netif, hostname.c_str());
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set hostname '%s': %s", hostname.c_str(), esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "hostname now %s (DHCP sends it on the next lease)", hostname.c_str());
}

esp_err_t WebServer::sendJson(httpd_req_t *req, json::Value doc, int status_code)
{
    const std::string body = json::print(doc);
    if (body.empty()) {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_status(req, statusLine(status_code));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body.c_str());
}

esp_err_t WebServer::sendError(httpd_req_t *req, int status_code, const std::string &message)
{
    json::Value doc = json::makeObject();
    if (!doc) {
        return httpd_resp_send_500(req);
    }
    json::addString(doc, "error", message);
    json::addNumber(doc, "statusCode", status_code);
    return sendJson(req, std::move(doc), status_code);
}

esp_err_t WebServer::notFound(httpd_req_t *req, httpd_err_code_t)
{
    return sendError(req, 404, "no such endpoint: " + std::string{req->uri});
}

esp_err_t WebServer::methodNotAllowed(httpd_req_t *req, httpd_err_code_t)
{
    return sendError(req, 405, "method not allowed on " + std::string{req->uri});
}

esp_err_t WebServer::healthz(httpd_req_t *req)
{
    json::Value doc = json::makeObject();
    if (!doc) {
        return httpd_resp_send_500(req);
    }

    json::addNumber(doc, "uptime", static_cast<double>(esp_timer_get_time() / 1000000ULL));
    json::addString(doc, "time", isoTimestamp(time(nullptr)));
    json::addBool(doc, "time_set", clockIsSet());

    if (const esp_app_desc_t *desc = esp_app_get_description()) {
        json::addString(doc, "version", desc->version);
        json::addString(doc, "idf_ver", desc->idf_ver);
    }
    const esp_partition_t *running = esp_ota_get_running_partition();
    json::addString(doc, "partition", running != nullptr ? running->label : "?");
    json::addNumber(doc, "heap_free", static_cast<double>(esp_get_free_heap_size()));

    const Settings &settings = Settings::instance();
    const NtpTime  &ntp      = NtpTime::instance();

    json::addString(doc, "hostname", settings.hostname());
    json::addBool(doc, "link_up", net_state::linkUp());

    esp_netif_ip_info_t ip_info;
    esp_netif_t        *netif = net_state::netif();
    if (netif != nullptr && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK &&
        ip_info.ip.addr != 0) {
        char text[16];
        snprintf(text, sizeof(text), IPSTR, IP2STR(&ip_info.ip));
        json::addString(doc, "ip", text);
    }

    /* The clock: where it is disciplined from, which upstream server is in use,
     * and whether it has actually been set rather than merely looking sane. */
    json::addString(doc, "ntp_source", ntp.sourceName());
    if (const std::optional<std::string> upstream = ntp.upstreamName()) {
        json::addString(doc, "ntp_server", *upstream);
        json::addBool(doc, "ntp_reachable", ntp.upstreamReachable());
    } else {
        json::addNull(doc, "ntp_server");
    }
    json::addBool(doc, "ntp_synced", ntp.synced());
    if (ntp.synced()) {
        json::addNumber(doc, "last_sync", static_cast<double>(ntp.lastSync()));
    }

    return sendJson(req, std::move(doc), 200);
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

    const std::optional<std::string> body = readBody(req);
    if (!body) {
        return sendError(req, 400, "request body truncated");
    }

    json::Value doc = json::parse(*body);
    if (!doc || !cJSON_IsObject(doc.get())) {
        return sendError(req, 400, "invalid JSON (expected an object)");
    }

    const std::string caller = callerAddress(req);

    Settings         &settings        = Settings::instance();
    const std::string hostname_before = settings.hostname();

    settings.applyJson(doc);
    settings.save();
    settings.log();

    if (settings.hostname() != hostname_before) {
        applyHostname(settings.hostname());
    }
    /* A changed timeSource/ntpServer takes effect now, not at the next boot. */
    NtpTime::instance().applySettings();
    ESP_LOGI(TAG, "POST /config from %s", caller.c_str());

    return sendJson(req, settings.toJson(), 200);
}

esp_err_t WebServer::configResetPost(httpd_req_t *req)
{
    const std::string caller = callerAddress(req);

    Settings &settings = Settings::instance();
    settings.resetDefaults();
    settings.save();
    settings.log();
    applyHostname(settings.hostname());
    NtpTime::instance().applySettings();
    ESP_LOGW(TAG, "POST /config/reset from %s: back to compiled-in defaults", caller.c_str());

    json::Value resp = json::makeObject();
    if (!resp) {
        return httpd_resp_send_500(req);
    }
    json::addString(resp, "status", "reset");
    return sendJson(req, std::move(resp), 200);
}

esp_err_t WebServer::syncPost(httpd_req_t *req)
{
    const std::string caller = callerAddress(req);

    NtpTime        &ntp = NtpTime::instance();
    const esp_err_t err = ntp.syncNow(pdMS_TO_TICKS(kSyncWaitMs));
    if (err == ESP_ERR_INVALID_STATE) {
        return sendError(req, 409, "no time source: set timeSource to dhcp or static");
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "POST /ntp/sync from %s: %s", caller.c_str(), esp_err_to_name(err));
        return sendError(req, 504, "no time sync from the upstream server");
    }

    const std::string now = isoTimestamp(time(nullptr));

    json::Value doc = json::makeObject();
    if (!doc) {
        return httpd_resp_send_500(req);
    }
    json::addBool(doc, "synced", true);
    json::addString(doc, "time", now);
    if (const std::optional<std::string> upstream = ntp.upstreamName()) {
        json::addString(doc, "ntp_server", *upstream);
    }
    ESP_LOGI(TAG, "POST /ntp/sync from %s: %s", caller.c_str(), now.c_str());

    return sendJson(req, std::move(doc), 200);
}

esp_err_t WebServer::registerRoutes()
{
    struct Route {
        const char    *uri;
        httpd_method_t method;
        esp_err_t    (*handler)(httpd_req_t *req);
    };

    static constexpr Route routes[] = {
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

    httpd_config_t config  = HTTPD_DEFAULT_CONFIG();
    config.server_port     = 80;
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


