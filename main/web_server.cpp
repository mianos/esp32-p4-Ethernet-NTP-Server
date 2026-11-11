#include "web_server.h"

#include <cstdio>
#include <ctime>
#include <optional>
#include <string>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"

#include "net_state.h"
#include "ntp_time.h"
#include "settings.h"

namespace {

constexpr const char *TAG = "webserver";

/* POST /ntp/sync blocks the request for at most this long. */
constexpr uint32_t kSyncWaitMs = 5000;

/* The handlers reach the one Settings/NtpTime through their singletons, so the
 * context carries nothing; WebServer just wants a non-null one for /healthz. */
WebContext s_web_context;

/* The RTC reads 1970 until something sets it, and this device has no RTC of
 * its own, so say which it is instead of presenting 1970 as a fact. */
bool clockIsSet()
{
    return time(nullptr) >= 1577836800; /* 2020-01-01T00:00:00Z */
}

}  // namespace

NtpWebServer &NtpWebServer::instance()
{
    static NtpWebServer server;
    return server;
}

NtpWebServer::NtpWebServer()
    : WebServer(&s_web_context)
{
}

esp_err_t NtpWebServer::begin()
{
    if (m_started) {
        return ESP_OK;
    }

    esp_err_t err = start(); /* /healthz, /config*, /ntp/sync, JSON 404/405 */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "http server failed to start: %s", esp_err_to_name(err));
        return err;
    }

    m_started = true;
    return ESP_OK;
}

void NtpWebServer::register_routes()
{
    WebServer::register_routes();

    struct Route {
        const char    *uri;
        httpd_method_t method;
        esp_err_t    (*handler)(httpd_req_t *req);
    };

    static constexpr Route routes[] = {
        {"/config",       HTTP_GET,  config_get_handler},
        {"/config",       HTTP_POST, config_post_handler},
        {"/config/reset", HTTP_POST, config_reset_post_handler},
        {"/ntp/sync",     HTTP_POST, sync_post_handler},
    };

    for (const Route &route : routes) {
        httpd_uri_t uri = {};
        uri.uri         = route.uri;
        uri.method      = route.method;
        uri.handler     = route.handler;
        uri.user_ctx    = webContext;

        esp_err_t err = httpd_register_uri_handler(server, &uri);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "register %s %s: %s",
                     route.method == HTTP_POST ? "POST" : "GET", route.uri,
                     esp_err_to_name(err));
            return;
        }
    }
}

/* The DHCP client embeds the hostname in its requests (option 12), but it only
 * builds them at DISCOVER/REQUEST time, so a change here reaches the lease on
 * the next renewal. */
void NtpWebServer::apply_hostname(const std::string &hostname)
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

void NtpWebServer::populate_healthz_fields(WebContext *, JsonWrapper &json)
{
    json.AddItem("time_set", clockIsSet());

    if (const esp_app_desc_t *desc = esp_app_get_description()) {
        json.AddItem("version", desc->version);
        json.AddItem("idf_ver", desc->idf_ver);
    }
    const esp_partition_t *running = esp_ota_get_running_partition();
    json.AddItem("partition", running != nullptr ? running->label : "?");
    json.AddItem("heap_free", static_cast<double>(esp_get_free_heap_size()));

    const Settings &settings = Settings::instance();
    const NtpTime  &ntp      = NtpTime::instance();

    json.AddItem("hostname", settings.hostname());
    json.AddItem("link_up", net_state::linkUp());

    esp_netif_ip_info_t ip_info;
    esp_netif_t        *netif = net_state::netif();
    if (netif != nullptr && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK &&
        ip_info.ip.addr != 0) {
        char text[16];
        snprintf(text, sizeof(text), IPSTR, IP2STR(&ip_info.ip));
        json.AddItem("ip", text);
    }

    /* The clock: where it is disciplined from, which upstream server is in use,
     * and whether it has actually been set rather than merely looking sane. */
    json.AddItem("ntp_source", ntp.sourceName());
    if (const std::optional<std::string> upstream = ntp.upstreamName()) {
        json.AddItem("ntp_server", *upstream);
        json.AddItem("ntp_reachable", ntp.upstreamReachable());
    } else {
        json.AddNull("ntp_server");
    }
    json.AddItem("ntp_synced", ntp.synced());
    if (ntp.synced()) {
        json.AddItem("last_sync", static_cast<double>(ntp.lastSync()));
    }
}

esp_err_t NtpWebServer::config_get_handler(httpd_req_t *req)
{
    return sendJson(req, Settings::instance().toJson(), 200);
}

esp_err_t NtpWebServer::config_post_handler(httpd_req_t *req)
{
    /* Shared helper: rejects an empty, oversized (>4 KB) or non-object body
     * with the JSON error shape already sent, and parses it otherwise. */
    JsonWrapper doc;
    if (!readJsonBody(req, doc)) {
        return ESP_FAIL;
    }

    const std::string caller = callerAddress(req);

    Settings         &settings        = Settings::instance();
    const std::string hostname_before = settings.hostname();

    settings.applyJson(doc);
    settings.save();
    settings.log();

    if (settings.hostname() != hostname_before) {
        apply_hostname(settings.hostname());
    }
    /* A changed timeSource/ntpServer takes effect now, not at the next boot. */
    NtpTime::instance().applySettings();
    ESP_LOGI(TAG, "POST /config from %s", caller.c_str());

    return sendJson(req, settings.toJson(), 200);
}

esp_err_t NtpWebServer::config_reset_post_handler(httpd_req_t *req)
{
    const std::string caller = callerAddress(req);

    Settings &settings = Settings::instance();
    settings.resetToDefaults(); /* fires the tz change callback */
    settings.save();
    settings.log();
    apply_hostname(settings.hostname());
    NtpTime::instance().applySettings();
    ESP_LOGW(TAG, "POST /config/reset from %s: back to compiled-in defaults", caller.c_str());

    JsonWrapper resp;
    resp.AddItem("status", "reset");
    return sendJson(req, resp, 200);
}

esp_err_t NtpWebServer::sync_post_handler(httpd_req_t *req)
{
    const std::string caller = callerAddress(req);

    NtpTime        &ntp = NtpTime::instance();
    const esp_err_t err = ntp.syncNow(pdMS_TO_TICKS(kSyncWaitMs));
    if (err == ESP_ERR_INVALID_STATE) {
        return sendJsonError(req, 409, "no time source: set timeSource to dhcp or static");
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "POST /ntp/sync from %s: %s", caller.c_str(), esp_err_to_name(err));
        return sendJsonError(req, 504, "no time sync from the upstream server");
    }

    const std::string now = isoTimestamp(time(nullptr));

    JsonWrapper doc;
    doc.AddItem("synced", true);
    doc.AddItem("time", now);
    if (const std::optional<std::string> upstream = ntp.upstreamName()) {
        doc.AddItem("ntp_server", *upstream);
    }
    ESP_LOGI(TAG, "POST /ntp/sync from %s: %s", caller.c_str(), now.c_str());

    return sendJson(req, doc, 200);
}

