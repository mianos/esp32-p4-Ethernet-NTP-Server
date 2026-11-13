/*
 * The device's HTTP surface: a WebServer subclass (mianesp's webserver
 * component), so the wire contract is the shared one:
 *
 *   GET  /healthz        uptime, clock, build, heap, link, lease, time source
 *   GET  /config         the settings JSON as stored
 *   POST /config         apply any subset of the keys, persist, echo the result
 *   POST /config/reset   restore compiled-in defaults
 *   POST /ntp/sync       force a poll of the upstream time server (test mode)
 *   POST /gps/ant        {"bias":"on"|"off"} switch antenna bias for a bench test
 *   POST /gps/reset      controlled hot start of the receiver
 *   POST /gps/pps        make the PPS interrupt ignore edges for a while, so a
 *                        GPS holdover can be measured on a board whose PPS line
 *                        is soldered (test mode; /healthz reports the time left)
 *   GET  /firmware       build and slot info for the running image
 *   POST /firmware       stream the body into the inactive OTA slot, then reboot
 *                        (this is how the device is updated over Ethernet; it
 *                        has no other field interface)
 *
 * Success is 2xx with the resulting state as the body; failure is
 * {"error": "...", "statusCode": N} with the matching HTTP status. Unknown
 * paths and wrong methods get the same JSON shape rather than httpd's built-in
 * plain-text page.
 *
 * WebServer supplies /healthz (uptime + time, then populate_healthz_fields()),
 * the JSON/body/caller helpers and the async workers. This class registers the
 * endpoints above and adds the device's own health fields. The WiFi endpoints
 * (/reset, /set_hostname) are deliberately absent: the P4 has no radio, so it
 * uses the network-agnostic base rather than wifiwebserver.
 */
#pragma once

#include <string>

#include "WebServer.h"

class NtpWebServer : public WebServer {
public:
    static NtpWebServer &instance();

    /* Start on :80, once, after the netif has an address. Idempotent. */
    esp_err_t begin();

private:
    NtpWebServer();

    void register_routes() override;
    void populate_healthz_fields(WebContext *ctx, JsonWrapper &json) override;

    static esp_err_t config_get_handler(httpd_req_t *req);
    static esp_err_t config_post_handler(httpd_req_t *req);
    static esp_err_t config_reset_post_handler(httpd_req_t *req);
    static esp_err_t sync_post_handler(httpd_req_t *req);
    static esp_err_t gps_pps_post_handler(httpd_req_t *req);
    static esp_err_t gps_ant_post_handler(httpd_req_t *req);
    static esp_err_t gps_reset_post_handler(httpd_req_t *req);
    static esp_err_t firmware_get_handler(httpd_req_t *req);
    static esp_err_t firmware_post_handler(httpd_req_t *req);

    /* Point the DHCP client's option-12 name at a renamed device. */
    static void apply_hostname(const std::string &hostname);

    bool m_started = false;
};
