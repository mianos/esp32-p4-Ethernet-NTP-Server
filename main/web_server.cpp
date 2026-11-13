#include "web_server.h"

#include <algorithm>
#include <cstdio>
#include <ctime>
#include <inttypes.h>
#include <optional>
#include <string>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gps_time.h"
#include "net_state.h"
#include "ntp_server.h"
#include "ntp_time.h"
#include "settings.h"

namespace {

constexpr const char *TAG = "webserver";

/* POST /ntp/sync blocks the request for at most this long. */
constexpr uint32_t kSyncWaitMs = 5000;

/* POST /gps/pps will not disable the PPS for longer than this. An hour is long
 * past the ten-minute holdover window, so a test can watch the reference demote
 * and then watch it recover; beyond that it stops being a test and starts being
 * a way to run a stratum-1 server with its reference switched off. */
constexpr int kMaxSuppressS = 3600;

/* POST /firmware moves the body a socket-chunk at a time into flash. Handlers
 * run on httpd's server task, and HTTPD_DEFAULT_CONFIG() gives that task 4 KB
 * of stack, so the buffer stays at 1 KB: enough to keep a 500 KB image quick
 * without eating the stack the rest of the handler needs. */
constexpr int kOtaChunkBytes = 1024;

/* Whatever the payload turns out to be, "this is not an app image for this
 * chip" is the caller's mistake, not the device's: IDF rejects it on the first
 * chunk (esp_ota_write checks the magic byte) or at esp_ota_end() (full image
 * verify), and both should read the same from outside. */
constexpr const char *kNotAnImage = "not a valid app image for this chip";

/* The handlers reach the one Settings/NtpTime through their singletons, so the
 * context carries nothing; WebServer just wants a non-null one for /healthz. */
WebContext s_web_context;

/* The RTC reads 1970 until something sets it, and this device has no RTC of
 * its own, so say which it is instead of presenting 1970 as a fact. */
bool clockIsSet()
{
    return time(nullptr) >= 1577836800; /* 2020-01-01T00:00:00Z */
}

/* The refid field is four bytes that mean different things depending on the
 * stratum: text ("GPS", "INIT", "RATE") at stratum 1 and for the kiss codes,
 * an IPv4 address above stratum 1 (RFC 5905 section 7.3). Print whichever it
 * is -- a hexadecimal blob answers none of the questions /healthz is asked. */
std::string refIdText(uint32_t refId, uint8_t stratum)
{
    char bytes[4];
    for (int i = 0; i < 4; ++i) {
        bytes[i] = static_cast<char>((refId >> (24 - 8 * i)) & 0xFF);
    }

    if (stratum <= 1 || stratum >= 16) {
        std::string text(bytes, sizeof(bytes));
        while (!text.empty() && text.back() == '\0') {
            text.pop_back();
        }
        return text;
    }

    char quad[16];
    std::snprintf(quad, sizeof(quad), "%u.%u.%u.%u", static_cast<uint8_t>(bytes[0]),
                  static_cast<uint8_t>(bytes[1]), static_cast<uint8_t>(bytes[2]),
                  static_cast<uint8_t>(bytes[3]));
    return quad;
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
        {"/gps/pps",      HTTP_POST, gps_pps_post_handler},
        {"/gps/ant",      HTTP_POST, gps_ant_post_handler},
        {"/gps/gnss",     HTTP_POST, gps_gnss_post_handler},
        {"/gps/info",     HTTP_GET,  gps_info_get_handler},
        {"/gps/reset",    HTTP_POST, gps_reset_post_handler},
        {"/firmware",     HTTP_GET,  firmware_get_handler},
        {"/firmware",     HTTP_POST, firmware_post_handler},
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

#if CONFIG_LWIP_IPV6
    /* The IPv6 half of the same story, and it does not arrive together with the
     * lease: the fe80:: address appears once DAD clears it, the global one only
     * after the router has answered a Router Solicitation. Each is omitted
     * until it exists, so a board with no RA (or an IPv4-only build) simply
     * does not have these keys. */
    if (const std::string link_local = net_state::ipv6LinkLocal(); !link_local.empty()) {
        json.AddItem("ipv6_linklocal", link_local);
    }
    if (const std::string global = net_state::ipv6Global(); !global.empty()) {
        json.AddItem("ipv6_global", global);
    }
#endif

    /* The clock: where it is disciplined from, which upstream server is in use,
     * and whether it has actually been set rather than merely looking sane.
     * With timeSource=gps there is no upstream to name: the receiver's own state
     * replaces that half, and ntp_synced means "the configured source is
     * locked", which for the GPS is a fix plus a fresh PPS edge. */
    json.AddItem("ntp_source", ntp.sourceName());
    if (ntp.source() == Settings::TimeSource::Gps) {
        const GpsTime::Status gps = GpsTime::instance().status();
        json.AddNull("ntp_server");
        json.AddItem("ntp_synced", gps.locked);
        if (gps.lastFix != 0) {
            json.AddItem("last_sync", static_cast<double>(gps.lastFix));
        }
        json.AddItem("gps_fix", static_cast<double>(gps.fixType));
        json.AddItem("gps_sats", static_cast<double>(gps.satellites));
        if (gps.ppsAgeUs >= 0) {
            json.AddItem("gps_pps_age_ms", static_cast<double>(gps.ppsAgeUs / 1000));
        }
        /* Only what answers "is it working": time, lock and satellites. Every
         * number behind those -- the servo, the signal, the antenna, the pulse
         * -- is at /gps/info, where a caller who is tuning something can get it
         * without a poller carrying it. The two configuration failures stay,
         * because a refused switch or pulse makes the rest meaningless. */
        if (gps.tp5Rejected) {
            json.AddItem("gps_tp5_rejected", true);
        }
        if (gps.antCfgRejected) {
            json.AddItem("gps_ant_cfg_rejected", true);
        }
        /* Present only when there is something wrong to say. */
        if (gps.antShort) {
            json.AddItem("gps_ant_short", true);
        }
        if (gps.antLatched) {
            json.AddItem("gps_ant_latched", true);
        }
        if (gps.ppsSuppressUs > 0) {
            json.AddItem("gps_pps_suppressed_s",
                         static_cast<double>(gps.ppsSuppressUs / 1000000));
        }
        /* What the last holdover actually cost, measured when the edge came
         * back: how long it was, and how far the servo's prediction was out.
         * Absent until there has been one. */
        if (gps.lastHoldoverUs > 0) {
            json.AddItem("gps_holdover_s", static_cast<double>(gps.lastHoldoverUs) / 1e6);
            json.AddItem("gps_holdover_error_us",
                         static_cast<double>(gps.lastHoldoverErrNs) / 1000.0);
        }
    } else {
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

    /* The responder's half of the story: whether it is listening, what it would
     * claim if asked right now, and how much it has answered since boot. */
    const NtpServer &server = NtpServer::instance();
    json.AddItem("serve_ntp", server.running());
    if (server.running()) {
        const TimeReference ref = server.reference();
        json.AddItem("ntp_stratum", static_cast<double>(ref.stratum));
        json.AddItem("ntp_refid", refIdText(ref.refId, ref.stratum));
        /* What the leap field of a reply would say right now: 1 or 2 for the
         * minute before a leap, 3 when the reference cannot vouch for the clock
         * at all, and 0 the rest of the time. */
        json.AddItem("ntp_leap", static_cast<double>(ref.locked ? ref.leap : 3));
        /* The same two numbers the reply's root delay and root dispersion
         * fields carry, before 16.16 rounds them: from the GPS, a measured
         * quantisation and jitter once NAV-TIM-TP is flowing, else the named
         * fallback; from an upstream nobody measured, the 0.992 ms LAN guess.
         * Dispersion grows with the age of the reference in both cases. */
        const NtpServer::RootFigures root = server.wireRootFigures();
        json.AddItem("ntp_root_delay_us", root.delayUs);
        json.AddItem("ntp_root_dispersion_us", root.dispersionUs);
        json.AddItem("ntp_requests", static_cast<double>(server.requests()));
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

    /* NVS is flash: only pay for a write when the config actually changed. */
    const std::string before = settings.toJson().ToString();
    settings.applyJson(doc);
    if (settings.toJson().ToString() != before) {
        settings.save();
    }
    settings.log();

    if (settings.hostname() != hostname_before) {
        apply_hostname(settings.hostname());
    }
    /* A changed timeSource/ntpServer takes effect now, not at the next boot. */
    NtpTime::instance().applySettings();
    /* And so does serveNtp: off stops the responder, on starts it. */
    NtpServer::instance().applySettings();
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
    NtpServer::instance().applySettings(); /* serveNtp is back to its default */
    ESP_LOGW(TAG, "POST /config/reset from %s: back to compiled-in defaults", caller.c_str());

    JsonWrapper resp;
    resp.AddItem("status", "reset");
    return sendJson(req, resp, 200);
}

esp_err_t NtpWebServer::sync_post_handler(httpd_req_t *req)
{
    const std::string caller = callerAddress(req);

    /* The GPS is not polled: it reports once a second whether anyone asks or
     * not, so "sync now" has nothing to trigger. It still has an answer worth
     * giving -- is the clock disciplined at this moment -- so the same reply
     * shape is built from the receiver's state rather than refused. */
    if (Settings::instance().timeSource() == Settings::TimeSource::Gps) {
        const GpsTime::Status gps = GpsTime::instance().status();
        if (!gps.locked) {
            ESP_LOGW(TAG, "POST /ntp/sync from %s: no GPS lock", caller.c_str());
            return sendJsonError(req, 504, "no GPS fix with PPS: the clock is not disciplined");
        }

        const std::string now = isoTimestamp(time(nullptr));
        JsonWrapper       doc;
        doc.AddItem("synced", true);
        doc.AddItem("time", now);
        doc.AddItem("source", "gps");
        doc.AddItem("gps_fix", static_cast<double>(gps.fixType));
        doc.AddItem("gps_sats", static_cast<double>(gps.satellites));
        ESP_LOGI(TAG, "POST /ntp/sync from %s: %s (GPS, %u satellites)", caller.c_str(),
                 now.c_str(), gps.satellites);
        return sendJson(req, doc, 200);
    }

    NtpTime        &ntp = NtpTime::instance();
    const esp_err_t err = ntp.syncNow(pdMS_TO_TICKS(kSyncWaitMs));
    if (err == ESP_ERR_INVALID_STATE) {
        return sendJsonError(req, 409, "no time source: set timeSource to dhcp, static or gps");
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

esp_err_t NtpWebServer::gps_pps_post_handler(httpd_req_t *req)
{
    /* The holdover test hook, and the reason it exists: the PPS line on this
     * board is soldered, so "unplug it and see what happens" is not available,
     * and a holdover nobody has exercised is a promise rather than a
     * measurement. This makes the interrupt ignore edges for a while; the
     * anchor stops advancing, dispersion grows, and the reference demotes
     * itself at the end of the window exactly as it would with a loose wire.
     * Nothing else changes, and /healthz reports the time left, so a reader can
     * tell a test from a fault. */
    JsonWrapper doc;
    if (!readJsonBody(req, doc)) {
        return ESP_FAIL;
    }

    int seconds = 0;
    if (!doc.GetField("suppress_s", seconds) || seconds < 0 || seconds > kMaxSuppressS) {
        return sendJsonError(req, 400, "suppress_s must be 0..3600, or 0 to end the test");
    }

    const std::string caller  = callerAddress(req);
    const uint32_t    applied = GpsTime::instance().suppressPps(static_cast<uint32_t>(seconds));
    const GpsTime::Status gps = GpsTime::instance().status();

    JsonWrapper resp;
    resp.AddItem("pps_suppressed_s", static_cast<double>(applied));
    resp.AddItem("locked", gps.locked);
    resp.AddItem("stratum", static_cast<double>(gps.locked ? 1 : 16));
    resp.AddItem("holdover_s", static_cast<double>(gps.ppsAgeUs > 0 ? gps.ppsAgeUs / 1000000 : 0));
    ESP_LOGW(TAG, "POST /gps/pps from %s: suppress_s=%u", caller.c_str(),
             static_cast<unsigned>(applied));
    return sendJson(req, resp, 200);
}

esp_err_t NtpWebServer::gps_info_get_handler(httpd_req_t *req)
{
    /* The static half of the receiver, out of /healthz because none of it
     * changes while the board is up and a poller should not have to carry it.
     * This is what a bench session reads: what the module says it is, which pins
     * it has, and which configuration messages it took. The ages come with it
     * because they are only meaningful next to the readings they date. */
    const GpsTime::Status gps = GpsTime::instance().status();

    JsonWrapper doc;
    if (!gps.moduleId.empty()) {
        doc.AddItem("module", gps.moduleId);
    }
    doc.AddItem("ant_cfg", gps.antCfgRejected ? "rejected" : gps.antCfgAcked ? "accepted"
                                                                             : "silent");
    doc.AddItem("tp5_ok", !gps.tp5Rejected);
    doc.AddItem("timtp", gps.timTpRejected ? "rejected" : gps.timTpAcked ? "accepted" : "silent");

    /* The servo and what it buys. */
    doc.AddItem("offset_us", static_cast<double>(gps.offsetUs));
    doc.AddItem("rate_ppm", static_cast<double>(gps.ratePpb) / 1000.0);
    doc.AddItem("rate_n", static_cast<double>(gps.rateSamples));
    doc.AddItem("fit_us", static_cast<double>(gps.fitUs));
    doc.AddItem("bridge_active", gps.bridgeActive);
    doc.AddItem("bridge_clients", static_cast<double>(gps.bridgeClients));
    doc.AddItem("bridge_sessions", static_cast<double>(gps.bridgeSessions));
    doc.AddItem("bridge_refused", static_cast<double>(gps.bridgeRefused));
    doc.AddItem("bridge_rx", static_cast<double>(gps.bridgeRx));
    doc.AddItem("bridge_tx", static_cast<double>(gps.bridgeTx));
    doc.AddItem("bridge_end", gps.bridgeEnd);
    doc.AddItem("bridge_last", gps.bridgeLast);
    doc.AddItem("gnss", gps.gnss);
    doc.AddItem("gnss_cfg", gps.gnssCfg);
    if (gps.locked) {
        doc.AddItem("dispersion_us", static_cast<double>(gps.dispersionNs) / 1000.0);
        doc.AddItem("lock_window_s", static_cast<double>(gps.lockWindowUs) / 1000000);
    }
    /* The pulse's own quality, once TIM-TP says anything. */
    if (gps.timTpCount != 0) {
        if (gps.qErrValid) {
            doc.AddItem("tp_qerr_ns", static_cast<double>(gps.qErrNs));
        }
        doc.AddItem("tp_jitter_ns", static_cast<double>(gps.jitterNs));
        doc.AddItem("tp_on_second", gps.pulseOnSecond);
    }
    /* The signal and the front end behind it: C/N0 is the antenna metric, and
     * agc/noise/jam are the overload tell-tales. */
    if (gps.monHwValid) {
        static constexpr const char *kAntStatus[] = {"init", "unknown", "ok", "short", "open"};
        static constexpr const char *kAntPower[]  = {"off", "on", "unknown"};
        static constexpr const char *kJam[]       = {"unknown", "ok", "warning", "critical"};
        doc.AddItem("ant", gps.antStatus <= 4 ? kAntStatus[gps.antStatus] : "unknown");
        doc.AddItem("ant_power", gps.antPower <= 2 ? kAntPower[gps.antPower] : "unknown");
        doc.AddItem("ant_bias", gps.antBiasOn ? "on" : "off");
        doc.AddItem("ant_short", gps.antShort);
        doc.AddItem("noise_dbm", -static_cast<double>(gps.noisePerMs));
        doc.AddItem("agc", static_cast<double>(gps.agcCnt));
        doc.AddItem("jam_ind", static_cast<double>(gps.jamInd));
        doc.AddItem("jam", gps.jamState <= 3 ? kJam[gps.jamState] : "unknown");
    }
    doc.AddItem("cn0_max", static_cast<double>(gps.cn0Max));
    doc.AddItem("cn0_mean", static_cast<double>(gps.cn0Mean));
    doc.AddItem("cn0_used", static_cast<double>(gps.cn0Used));
    /* The leap-second notice and the last holdover, both diagnostics. */
    if (gps.leapValid) {
        doc.AddItem("leap_s", static_cast<double>(gps.leapSeconds));
        const bool pending = gps.leapChange != 0 && gps.leapInS > 0;
        doc.AddItem("leap_pending", pending);
        if (pending) {
            doc.AddItem("leap_in_s", static_cast<double>(gps.leapInS));
        }
    }
    if (gps.lastHoldoverUs > 0) {
        doc.AddItem("holdover_s", static_cast<double>(gps.lastHoldoverUs) / 1e6);
        doc.AddItem("holdover_error_us", static_cast<double>(gps.lastHoldoverErrNs) / 1000.0);
    }

    if (gps.monHwValid) {
        /* Hex strings: these are bit maps, not quantities. */
        char bits[32];
        std::snprintf(bits, sizeof(bits), "%08lx", static_cast<unsigned long>(gps.pinSel));
        doc.AddItem("pin_sel", std::string(bits));
        std::snprintf(bits, sizeof(bits), "%08lx", static_cast<unsigned long>(gps.pinDir));
        doc.AddItem("pin_dir", std::string(bits));
        std::snprintf(bits, sizeof(bits), "%08lx", static_cast<unsigned long>(gps.pinVal));
        doc.AddItem("pin_val", std::string(bits));
        doc.AddItem("mon_hw_age_s", static_cast<double>(gps.monHwAgeUs) / 1e6);
    }
    if (gps.navSatAgeUs >= 0) {
        doc.AddItem("cn0_age_s", static_cast<double>(gps.navSatAgeUs) / 1e6);
    }
    return sendJson(req, doc, 200);
}

esp_err_t NtpWebServer::gps_ant_post_handler(httpd_req_t *req)
{
    /* The antenna bench hook. Swapping antennas is the one job where "is the
     * bias on" has to be answerable and changeable from outside, and where the
     * two antennas must be compared under the *same* settings -- so this
     * switches the supply deliberately rather than leaving it to whatever the
     * module was last left set to. The C/N0 either side of the change is the
     * comparison; gps_cn0_* reports it and nothing here interprets it. */
    JsonWrapper doc;
    if (!readJsonBody(req, doc)) {
        return ESP_FAIL;
    }

    std::string bias;
    if (!doc.GetField("bias", bias) || (bias != "on" && bias != "off")) {
        return sendJsonError(req, 400, "bias must be \"on\" or \"off\"");
    }

    GpsTime::instance().requestAntennaBias(bias == "on");
    ESP_LOGW(TAG, "POST /gps/ant from %s: bias %s", callerAddress(req).c_str(), bias.c_str());

    /* Answered before it has happened: the GPS task owns the UART, so the
     * change is queued for its next pass. Say so, and say when to look. */
    JsonWrapper resp;
    resp.AddItem("requested", bias);
    resp.AddItem("read_back_after_s", 3.0);
    resp.AddItem("note", "drives GPIO17; gps_ant_bias and gps_ant_short report the result");
    return sendJson(req, resp, 200);
}

esp_err_t NtpWebServer::gps_gnss_post_handler(httpd_req_t *req)
{
    /* Which constellations the receiver solves. This is a switch rather than a
     * fixed choice because the right answer is site-dependent, and because the
     * receiver will not take them all.
     *
     * BeiDou and GLONASS are mutually exclusive here: enabling BeiDou is NAKed
     * while GLONASS is on, and NAKed while Galileo is on for the same reason.
     * So the two coherent sets are "max" (GPS + Galileo + GLONASS + QZSS, 37
     * satellites in view here, median elevation 49 degrees) and "beidou"
     * (GPS + BeiDou + QZSS, which trades GLONASS and Galileo for BeiDou). A
     * raw mask is accepted too, for anything neither name covers, and a mask
     * the receiver refuses is reported back rather than silently kept. */
    JsonWrapper doc;
    if (!readJsonBody(req, doc)) {
        return ESP_FAIL;
    }

    /* gnssId order: GPS 0, SBAS 1, Galileo 2, BeiDou 3, IMES 4, QZSS 5,
     * GLONASS 6. SBAS and IMES are off in both named sets: nothing applies
     * SBAS corrections and IMES is indoor-only. */
    std::string set;
    uint8_t     mask = 0;
    if (doc.GetField("set", set)) {
        if (set == "max") {
            mask = (1u << 0) | (1u << 2) | (1u << 5) | (1u << 6);
        } else if (set == "beidou") {
            mask = (1u << 0) | (1u << 3) | (1u << 5);
        } else {
            return sendJsonError(req, 400, "set must be \"max\" or \"beidou\"");
        }
    } else {
        double raw = 0;
        if (!doc.GetField("mask", raw) || raw < 1 || raw > 127) {
            return sendJsonError(req, 400, "need {\"set\":\"max\"|\"beidou\"} or {\"mask\":1..127}");
        }
        mask = static_cast<uint8_t>(raw);
    }

    GpsTime::instance().requestGnss(mask);
    ESP_LOGW(TAG, "POST /gps/gnss from %s: mask 0x%02x", callerAddress(req).c_str(),
             static_cast<unsigned>(mask));

    JsonWrapper resp;
    resp.AddItem("requested_mask", static_cast<double>(mask));
    resp.AddItem("read_back_after_s", 20.0);
    resp.AddItem("note", "gps_gnss and gps_gnss_cfg at /gps/info report what was set and "
                         "what the receiver said; a mask it refuses leaves the old set");
    return sendJson(req, resp, 200);
}

esp_err_t NtpWebServer::gps_reset_post_handler(httpd_req_t *req)
{
    /* A controlled hot start. The receiver gets reconfigured on every boot and
     * has been worked hard on a bench; this restarts the navigation engine
     * without clearing the almanac, which is the cheap way back from a state it
     * will not leave on its own. */
    /* The body is optional, so an empty POST still means the cheap thing. */
    bool cold = false;
    bool hard = false;
    bool power = false;
    bool defaults = false;
    const int len = req->content_len;
    if (len > 0 && len < 512) {
        char buf[512];
        const int got = httpd_req_recv(req, buf, len);
        if (got > 0) {
            JsonWrapper doc = JsonWrapper::Parse(std::string(buf, static_cast<size_t>(got)));
            if (!doc.Empty()) {
                doc.GetField("cold", cold);
                doc.GetField("hardware", hard);
                doc.GetField("power", power);
                doc.GetField("defaults", defaults);
            }
        }
    }

    if (defaults) {
        GpsTime::instance().requestDefaults();
    } else {
        GpsTime::instance().requestReset(cold, hard, power);
    }
    ESP_LOGW(TAG, "POST /gps/reset from %s: %s%s%s start", callerAddress(req).c_str(),
             power ? "power-cycle " : "", hard ? "hardware " : "", cold ? "cold" : "hot");

    JsonWrapper resp;
    resp.AddItem("reset", defaults ? "configuration cleared to defaults"
                                 : power ? "power-cycle reset queued"
                                 : hard ? "hardware reset queued"
                                        : cold ? "cold start queued" : "hot start queued");
    resp.AddItem("note", cold ? "the almanac is discarded; expect a slow reacquisition"
                              : "the almanac is kept; send {\"cold\":true} to discard it");
    return sendJson(req, resp, 200);
}

esp_err_t NtpWebServer::firmware_get_handler(httpd_req_t *req)
{
    const esp_app_desc_t  *desc    = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *target  = esp_ota_get_next_update_partition(nullptr);

    JsonWrapper doc;
    if (desc != nullptr) {
        doc.AddItem("version", desc->version);
        doc.AddItem("idf_ver", desc->idf_ver);
        doc.AddItem("built", std::string(desc->date) + " " + desc->time);
    }
    doc.AddItem("partition", running != nullptr ? running->label : "?");

    /* The rollback handshake, visible from outside: a freshly written image is
     * PENDING_VERIFY until main.cpp confirms it, and an unconfirmed one is
     * dropped on the next reset in favour of the previous slot. */
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    doc.AddItem("confirmed", running != nullptr &&
                                 esp_ota_get_state_partition(running, &state) == ESP_OK &&
                                 state == ESP_OTA_IMG_VALID);

    /* `ota` false means this table has no second slot, so POST /firmware would
     * be refused and updating needs the USB cable again. */
    doc.AddItem("ota", target != nullptr);
    if (target != nullptr) {
        doc.AddItem("next_partition", target->label);
        doc.AddItem("slot_size", static_cast<double>(target->size));
    } else {
        doc.AddNull("next_partition");
    }

    return sendJson(req, doc, 200);
}

/* Writes the body into the inactive slot and boots it. The only check on the
 * payload is esp_ota_end()'s: a well-formed app image for this chip. An image
 * that is valid but wrong still lands in the slot, which is what the rollback
 * handshake in main.cpp is for -- if it never gets a lease, the bootloader
 * returns to the slot that worked. Unauthenticated, like every endpoint here:
 * this firmware assumes a trusted LAN. */
esp_err_t NtpWebServer::firmware_post_handler(httpd_req_t *req)
{
    if (req->content_len <= 0) {
        return sendJsonError(req, 400, "Content-Length required");
    }

    const esp_partition_t *target = esp_ota_get_next_update_partition(nullptr);
    if (target == nullptr) {
        return sendJsonError(req, 500, "no OTA partition: flash a table with two app slots");
    }
    if (static_cast<uint32_t>(req->content_len) > target->size) {
        return sendJsonError(req, 413, "image larger than the OTA partition");
    }

    const std::string caller = callerAddress(req);
    ESP_LOGW(TAG, "POST /firmware from %s: %d bytes -> %s @ 0x%08" PRIx32, caller.c_str(),
             req->content_len, target->label, target->address);

    esp_ota_handle_t handle = 0;
    esp_err_t        err    = esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err == ESP_ERR_OTA_ROLLBACK_INVALID_STATE) {
        /* This image has not confirmed itself yet, so it is not allowed to
         * write another one. Retrying in a few seconds is enough. */
        return sendJsonError(req, 409, "running image unconfirmed: retry once it has a lease");
    }
    if (err != ESP_OK) {
        return sendJsonError(req, 500, esp_err_to_name(err));
    }

    char buf[kOtaChunkBytes];
    int  remaining = req->content_len;
    int  written   = 0;
    while (remaining > 0) {
        const int got = httpd_req_recv(req, buf, std::min(remaining, kOtaChunkBytes));
        if (got == HTTPD_SOCK_ERR_TIMEOUT) {
            continue; /* httpd already waited its recv timeout; keep going */
        }
        if (got <= 0) {
            esp_ota_abort(handle);
            return sendJsonError(req, 400, "request body truncated");
        }
        err = esp_ota_write(handle, buf, got);
        if (err != ESP_OK) {
            esp_ota_abort(handle);
            if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
                return sendJsonError(req, 400, kNotAnImage);
            }
            return sendJsonError(req, 500, esp_err_to_name(err));
        }
        written   += got;
        remaining -= got;
    }

    err = esp_ota_end(handle); /* re-reads the header and validates the image */
    if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
        return sendJsonError(req, 400, kNotAnImage);
    }
    if (err != ESP_OK) {
        return sendJsonError(req, 500, esp_err_to_name(err));
    }

    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
        return sendJsonError(req, 500, esp_err_to_name(err));
    }
    ESP_LOGW(TAG, "OTA: %d bytes written to %s; rebooting into it", written, target->label);

    JsonWrapper doc;
    doc.AddItem("written", written);
    doc.AddItem("partition", target->label);
    doc.AddItem("rebooting", true);
    sendJson(req, doc, 200);

    /* Give the reply time to leave the socket before the reboot drops it. */
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}


