/* The HTTP API, on mianesp's webserver component (no WiFi endpoints).
 *
 *   GET  /healthz        health summary
 *   GET  /config         settings as stored
 *   POST /config         apply any subset of keys, persist, echo the result
 *   POST /config/reset   restore defaults
 *   POST /ntp/sync       poll the upstream now
 *   POST /ntp/txlag      switch the sendto() transmit correction on or off
 *   GET  /gps/info       receiver diagnostics, the polled subset
 *   GET  /gps/bench      receiver diagnostics, everything
 *   GET  /gps/sats       per-satellite sky view
 *   GET  /gps/samples    the per-second sample ring, by cursor
 *   POST /gps/ant        antenna bias, cable and user delay, pulse width, period
 *   POST /gps/gnss       constellation set
 *   POST /gps/survey     start or stop a survey, or hold its position
 *   POST /gps/reset      receiver reset
 *   POST /gps/baud       change the UART rate
 *   POST /gps/pps        ignore PPS edges for a while (holdover test)
 *   POST /eth/restart    restart the MAC
 *   GET  /firmware       running image and slots
 *   POST /firmware       write the body to the other OTA slot and reboot
 *
 * Errors are {"error": "...", "statusCode": N} with that HTTP status, including
 * for unknown paths and methods. */
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
    static esp_err_t txlag_post_handler(httpd_req_t *req);
    static esp_err_t gps_pps_post_handler(httpd_req_t *req);
    static esp_err_t gps_ant_post_handler(httpd_req_t *req);
    static esp_err_t gps_gnss_post_handler(httpd_req_t *req);
    static esp_err_t gps_survey_post_handler(httpd_req_t *req);
    static esp_err_t gps_info_get_handler(httpd_req_t *req);
    static esp_err_t gps_bench_get_handler(httpd_req_t *req);
    static esp_err_t gps_sats_get_handler(httpd_req_t *req);
    static esp_err_t gps_samples_get_handler(httpd_req_t *req);
    static esp_err_t gps_reset_post_handler(httpd_req_t *req);
    static esp_err_t gps_baud_post_handler(httpd_req_t *req);
    static esp_err_t eth_restart_post_handler(httpd_req_t *req);
    static esp_err_t firmware_get_handler(httpd_req_t *req);
    static esp_err_t firmware_post_handler(httpd_req_t *req);

    /* Read a JSON body like readJsonBody(), which loops for ever on a socket
     * timeout, but give up after kBodyIdleUs without progress or kBodyTotalUs. */
    static bool readJsonBodyTimed(httpd_req_t *req, JsonWrapper &out);

    bool m_started = false;
};
