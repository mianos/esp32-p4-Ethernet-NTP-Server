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
#include "net_log.h"
#include "ntp_server.h"
#include "ntp_time.h"
#include "settings.h"
#include "text.h"

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

/* esp_reset_reason() as text. BROWNOUT and POWERON are what the mains looks
 * like; PANIC and TASK_WDT are the firmware's own doing, and conflating them
 * wastes an afternoon. */
const char *resetReasonText()
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_BROWNOUT: return "brown-out";
    case ESP_RST_EXT:      return "external-pin";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "panic";
    case ESP_RST_INT_WDT:  return "interrupt-watchdog";
    case ESP_RST_TASK_WDT: return "task-watchdog";
    case ESP_RST_WDT:      return "other-watchdog";
    case ESP_RST_DEEPSLEEP: return "deep-sleep";
    case ESP_RST_SDIO:     return "sdio";
    default:               return "unknown";
    }
}

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

    return std::format("{}.{}.{}.{}", static_cast<uint8_t>(bytes[0]),
                       static_cast<uint8_t>(bytes[1]), static_cast<uint8_t>(bytes[2]),
                       static_cast<uint8_t>(bytes[3]));
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
        {"/gps/survey",   HTTP_POST, gps_survey_post_handler},
        {"/gps/info",     HTTP_GET,  gps_info_get_handler},
        {"/gps/sats",     HTTP_GET,  gps_sats_get_handler},
        {"/gps/reset",    HTTP_POST, gps_reset_post_handler},
        {"/gps/baud",     HTTP_POST, gps_baud_post_handler},
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
    /* Why this boot happened, and how many there have been. A power cut used to
     * be inferable only from what had silently gone missing. */
    json.AddItem("reset_reason", resetReasonText());
    json.AddItem("boots", std::strtoul(Settings::instance().boots().c_str(), nullptr, 10));
    json.AddItem("heap_free", static_cast<double>(esp_get_free_heap_size()));

    const Settings &settings = Settings::instance();
    const NtpTime  &ntp      = NtpTime::instance();

    json.AddItem("hostname", settings.hostname());
    json.AddItem("link_up", net_state::linkUp());

    esp_netif_ip_info_t ip_info;
    esp_netif_t        *netif = net_state::netif();
    if (netif != nullptr && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK &&
        ip_info.ip.addr != 0) {
        /* IPSTR/IP2STR are lwIP's own printf macro, expanded rather than
         * reformatted: the three fields are already in the format's order. */
        const uint8_t *o = reinterpret_cast<const uint8_t *>(&ip_info.ip.addr);
        json.AddItem("ip", std::format("{}.{}.{}.{}", o[0], o[1], o[2], o[3]));
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
        /* Time validity, edge count and whether an edge has ever been paired
         * with a date. A 3D fix with invalid time is the state that quietly
         * serves rubbish, and `locked` alone hides it by folding all three
         * together; with ppsCount the difference between "the edges stopped"
         * and "the edges never started" also stops reading the same. */
        json.AddItem("gps_time_valid", gps.timeValid);
        json.AddItem("gps_pps_count", static_cast<double>(gps.ppsCount));
        json.AddItem("gps_anchored", gps.anchored);
        if (gps.ppsAgeUs >= 0) {
            json.AddItem("gps_pps_age_ms", static_cast<double>(gps.ppsAgeUs / 1000));
        }
        /* Framing health. The answer and the receiver going quiet look the same
         * from every other field here, and these three tell them apart: good
         * frames, frames the checksum or the length table threw away, and how
         * often the line has been re-scanned for the baud. */
        json.AddItem("gps_frames", static_cast<double>(gps.frames));
        json.AddItem("gps_dropped_crc", static_cast<double>(gps.droppedChecksum));
        json.AddItem("gps_dropped_len", static_cast<double>(gps.droppedLength));
        json.AddItem("gps_rescans", static_cast<double>(gps.rescans));
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
        /* How the requests were treated, not just how many there were. A client
         * being rate-limited and a client that stopped asking both leave
         * `ntp_requests` flat; the minute rate and the two rejection counters
         * separate them, and lastServed says how long the flat has lasted. */
        const NtpServer::Counters counters = server.counters();
        json.AddItem("ntp_answered", static_cast<double>(counters.answered));
        json.AddItem("ntp_kod", static_cast<double>(counters.kod));
        json.AddItem("ntp_dropped", static_cast<double>(counters.dropped));
        json.AddItem("ntp_rate_per_min", counters.perMinute);
        json.AddItem("ntp_clients", static_cast<double>(counters.clients));
        if (counters.lastServedS >= 0) {
            json.AddItem("ntp_last_served_s", counters.lastServedS);
        }
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

    /* Started here rather than from an onChange hook: applyJson also runs at
     * boot during load, before lwIP is up. */
    if (doc.ContainsField("logHost") || doc.ContainsField("logPort")) {
        net_log::start();
    }
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
    doc.AddItem("offset_ns", static_cast<double>(gps.offsetNs));
    doc.AddItem("rate_ppm", static_cast<double>(gps.ratePpb) / 1000.0);
    doc.AddItem("rate_n", static_cast<double>(gps.rateSamples));
    doc.AddItem("fit_us", static_cast<double>(gps.fitUs));
    doc.AddItem("bridge_active", gps.bridgeActive);
    doc.AddItem("bridge_clients", static_cast<double>(gps.bridgeClients));
    doc.AddItem("bridge_sessions", static_cast<double>(gps.bridgeSessions));
    doc.AddItem("bridge_refused", static_cast<double>(gps.bridgeRefused));
    doc.AddItem("bridge_dropped_crc", static_cast<double>(gps.bridgeDroppedCrc));
    doc.AddItem("bridge_rx", static_cast<double>(gps.bridgeRx));
    doc.AddItem("bridge_tx", static_cast<double>(gps.bridgeTx));
    doc.AddItem("bridge_end", gps.bridgeEnd);
    doc.AddItem("bridge_last", gps.bridgeLast);
    doc.AddItem("gnss", gps.gnss);
    doc.AddItem("gnss_cfg", gps.gnssCfg);
    doc.AddItem("time_mode", gps.timeMode);
    if (gps.svinAgeS >= 0) {
        doc.AddItem("svin_dur_s", gps.svinDurS);
        doc.AddItem("svin_acc_mm", gps.svinAccMm);
        doc.AddItem("svin_var_mm2", gps.svinVarMm2);
        doc.AddItem("svin_obs", gps.svinObs);
        doc.AddItem("svin_target_s", gps.svinTargetS);
        doc.AddItem("svin_limit_mm", gps.svinLimitMm);
        doc.AddItem("svin_valid", gps.svinValid);
        doc.AddItem("svin_age_s", gps.svinAgeS);
    }
    if (gps.locked) {
        doc.AddItem("dispersion_us", static_cast<double>(gps.dispersionNs) / 1000.0);
        doc.AddItem("lock_window_s", static_cast<double>(gps.lockWindowUs) / 1000000);
    }
    /* The pulse's own quality. The jitter is OURS -- the spread of our own
     * stamping from the rate fit -- and was gated behind timTpCount with the two
     * TIM-TP figures, which hid it forever on a module that refuses the message
     * (this one does). qErr and onSecond genuinely need TIM-TP; the jitter does
     * not, so it stands alone. */
    doc.AddItem("tp_jitter_ns", static_cast<double>(gps.jitterNs));
    if (gps.tosCfg != 0) {
        doc.AddItem("tos_cfg", gps.tosCfg == 1 ? "accepted" : "rejected");
    }
    if (gps.tosFlags != 0) {
        doc.AddItem("tos_locked", (gps.tosFlags & (1u << 13)) != 0);
        doc.AddItem("tos_coherent", (gps.tosFlags & (1u << 12)) != 0);
        doc.AddItem("tos_utc_offset_ns", static_cast<double>(gps.tosUtcOffsetNs));
        doc.AddItem("tos_utc_unc_ns", static_cast<double>(gps.tosUtcUncNs));
    }
    if (gps.timTpCount != 0) {
        if (gps.qErrValid) {
            doc.AddItem("tp_qerr_ns", static_cast<double>(gps.qErrNs));
        }
        doc.AddItem("tp_on_second", gps.pulseOnSecond);
        /* The pulse's own position in the second, as the receiver states it.
         * Constant here means every anchor, and so the served time, is shifted
         * by it; nothing downstream can see a constant. */
        doc.AddItem("tp_offset_ns", static_cast<double>(gps.pulseOffsetNs));
        /* The solution's own offset into the second it names. If this tracks the
         * disagreement with outside time, the receiver's UTC is displaced from
         * its pulse and no amount of servo work can see it. */
        doc.AddItem("pvt_nano_ns", static_cast<double>(gps.pvtNanoNs));
        /* The same instant on the GPS time base. GPS and UTC differ only by
         * whole leap seconds, so the fractional parts of this and of pvt_nano
         * must agree: to within the solution, they are the same epoch. A
         * disagreement is the receiver's UTC displaced from its own GNSS time,
         * and that needs nothing outside the receiver to see. */
        doc.AddItem("pvt_itow_ms", static_cast<double>(gps.pvtItowMs % 1000));
        /* bit 0 time base (0 = GNSS, 1 = UTC), bit 3 qErr invalid. */
        doc.AddItem("timtp_flags", static_cast<double>(gps.timTpFlags));
    }

    /* The servo's work counters, and the measurement-path half of the error
     * budget. A mean offset cannot tell a settled servo from one that is
     * fighting: steps and clamps are how often it has had to act, and spikes
     * are edges that arrived where nothing predicted. */
    doc.AddItem("root_delay_ns", static_cast<double>(gps.rootDelayNs));
    doc.AddItem("servo_steps", static_cast<double>(gps.servoSteps));
    doc.AddItem("servo_clamps", static_cast<double>(gps.servoClamps));
    doc.AddItem("edge_spikes", static_cast<double>(gps.edgeSpikes));
    doc.AddItem("tie_max_ns", gps.tieMaxNs);
    doc.AddItem("tie_rms_ns", gps.tieRmsNs);

    /* The Allan deviation over the fit window, in nanoseconds per second --
     * which is ppb, and is what the arithmetic already returns because the
     * phase is in nanoseconds over one-second samples. Published in those units
     * rather than fractionally because the JSON writer rounds to four decimals
     * and 1e-10 would arrive as 0.0000. Gated on the sample count because below
     * ~60 s into a window the short octaves still move by a factor between
     * polls, and a viewer plotting that reads as instability that is not there.
     * A zero octave means the window does not reach it. */
    if (gps.adevN >= 60) {
        static constexpr const char *kAdevKeys[GpsTime::Status::kAdevOctaves] = {
            "adev_1s", "adev_2s", "adev_4s", "adev_8s", "adev_16s", "adev_32s", "adev_64s"};
        for (int k = 0; k < GpsTime::Status::kAdevOctaves; ++k) {
            if (gps.adev[k] > 0.0) {
                doc.AddItem(kAdevKeys[k], gps.adev[k]);
            }
        }
        doc.AddItem("adev_n", gps.adevN);
    }

    /* Geometry and clock quality. Each carries its own age so a message that
     * stops is visible as a growing number rather than a stale one that still
     * reads current. */
    if (gps.dopAgeUs >= 0) {
        doc.AddItem("gdop", static_cast<double>(gps.gDop));
        doc.AddItem("pdop", static_cast<double>(gps.pDop));
        doc.AddItem("hdop", static_cast<double>(gps.hDop));
        doc.AddItem("vdop", static_cast<double>(gps.vDop));
        doc.AddItem("dop_age_s", static_cast<double>(gps.dopAgeUs) / 1e6);
    }
    if (gps.clockAgeUs >= 0) {
        doc.AddItem("clk_bias_ns", static_cast<double>(gps.clkBiasNs));
        doc.AddItem("clk_drift_ns_s", static_cast<double>(gps.clkDriftNsPerS));
        doc.AddItem("t_acc_ns", static_cast<double>(gps.tAccNs));
        doc.AddItem("f_acc_ps_s", static_cast<double>(gps.fAccPsPerS));
        doc.AddItem("clock_age_s", static_cast<double>(gps.clockAgeUs) / 1e6);
    }
    if (gps.navStatusAgeUs >= 0) {
        doc.AddItem("nav_status_flags", static_cast<double>(gps.navStatusFlags));
        doc.AddItem("nav_status_flags2", static_cast<double>(gps.navStatusFlags2));
        doc.AddItem("ttff_ms", static_cast<double>(gps.ttffMs));
        doc.AddItem("nav_status_age_s", static_cast<double>(gps.navStatusAgeUs) / 1e6);
    }
    /* The link the diagnostics cross, and whether it is coping. All byte
     * counters are cumulative, so a rate needs two samples; the parity,
     * framing and overrun figures are the wire's own verdict and are worth
     * reading next to the decoder's CRC drops, which they precede. */
    /* Ungated on purpose: these are what say whether the link is working at
     * all, so they must not sit behind a message that the trouble may have
     * stopped. */
    doc.AddItem("baud", static_cast<double>(gps.baud));
    doc.AddItem("ticks_per_us", static_cast<double>(gps.ticksPerUs));
    doc.AddItem("tick_us_delta", static_cast<double>(gps.tickUsDelta));
    doc.AddItem("recent_frames", gps.recentFrames);
    {
        doc.AddItem("baud_dup", static_cast<double>(gps.baud));
        if (gps.ioSeen && gps.ioAgeUs >= 0) {
            doc.AddItem("io_rx_bytes", static_cast<double>(gps.ioRxBytes));
            doc.AddItem("io_tx_bytes", static_cast<double>(gps.ioTxBytes));
            doc.AddItem("io_parity", static_cast<double>(gps.ioParity));
            doc.AddItem("io_framing", static_cast<double>(gps.ioFraming));
            doc.AddItem("io_overrun", static_cast<double>(gps.ioOverrun));
            doc.AddItem("rbuf_pending", static_cast<double>(gps.rbufPending));
            doc.AddItem("rbuf_peak", static_cast<double>(gps.rbufPeak));
            doc.AddItem("tbuf_pending", static_cast<double>(gps.tbufPending));
            doc.AddItem("tbuf_peak", static_cast<double>(gps.tbufPeak));
            doc.AddItem("tbuf_errors", static_cast<double>(gps.tbufErrors));
            doc.AddItem("msg_ubx", static_cast<double>(gps.msgUbx));
            doc.AddItem("msg_skipped", static_cast<double>(gps.msgSkipped));
            doc.AddItem("msgpp_hex", gps.msgppHex);
            doc.AddItem("io_age_s", static_cast<double>(gps.ioAgeUs) / 1e6);
            doc.AddItem("io_hex", gps.ioHex);
        }
    }

    /* MON-RF's fate is always reported, even when it is carrying nothing: a
     * missing field and a refused message look identical otherwise, and this
     * module's protocol predates the message. */
    {
        static constexpr const char *kRfCfg[] = {"silent", "accepted", "rejected"};
        doc.AddItem("rf_cfg", gps.rfCfg <= 2 ? kRfCfg[gps.rfCfg] : "silent");
    }
    if (gps.rfSeen && gps.rfAgeUs >= 0) {
        doc.AddItem("rf_bands", static_cast<double>(gps.rfBands));
        doc.AddItem("rf_jam_worst", static_cast<double>(gps.rfJamWorst));
        doc.AddItem("rf_ant_worst", static_cast<double>(gps.rfAntWorst));
        doc.AddItem("rf_agc_min", static_cast<double>(gps.rfAgcMin));
        doc.AddItem("rf_age_s", static_cast<double>(gps.rfAgeUs) / 1e6);
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
        doc.AddItem("ant_delay_ns", gps.antDelayNs);
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
        doc.AddItem("pin_sel", std::format("{:08x}", gps.pinSel));
        doc.AddItem("pin_dir", std::format("{:08x}", gps.pinDir));
        doc.AddItem("pin_val", std::format("{:08x}", gps.pinVal));
        doc.AddItem("mon_hw_age_s", static_cast<double>(gps.monHwAgeUs) / 1e6);
    }
    if (gps.navSatAgeUs >= 0) {
        doc.AddItem("cn0_age_s", static_cast<double>(gps.navSatAgeUs) / 1e6);
    }
    return sendJson(req, doc, 200);
}

esp_err_t NtpWebServer::gps_sats_get_handler(httpd_req_t *req)
{
    /* Per-satellite C/N0 with elevation and azimuth, for a polar sky view.
     * Separate from /gps/info because this is the one response whose size
     * follows the sky: 32 satellites is a couple of kilobytes, and folding it
     * into the info dump would make every poller pay for it.
     *
     * A plot is worth the space because the aggregate cannot separate the two
     * ways a satellite can be bad. One reflected off a roof arrives loud and
     * late; one low in the sky is simply weak. Both land in the same cn0Mean,
     * and only elevation against C/N0 -- with the healthy and svUsed flags
     * beside them -- tells which is which. That difference is what decides
     * whether the site or the antenna is at fault, which is the question a
     * survey that will not converge asks.
     *
     * flags is passed through as the receiver's own word rather than being
     * unpacked here: qualityInd is bits 0-2, svUsed 3, health 4, ephAvail 8,
     * and a viewer that wants them can mask. ephAvail is unscaled because a
     * satellite being tracked without ephemeris is a different fault from one
     * being tracked badly. */
    const GpsTime::Status gps = GpsTime::instance().status();

    cJSON *root = cJSON_CreateObject();
    if (root == nullptr) {
        return sendJsonError(req, 500, "out of memory");
    }
    cJSON_AddNumberToObject(root, "used", gps.cn0Used);
    cJSON_AddNumberToObject(root, "mean", gps.cn0Mean);
    cJSON_AddNumberToObject(root, "max", gps.cn0Max);
    if (gps.navSatAgeUs >= 0) {
        cJSON_AddNumberToObject(root, "age_ms", static_cast<double>(gps.navSatAgeUs) / 1000.0);
    } else {
        cJSON_AddNullToObject(root, "age_ms");
    }

    /* The rows live here, on the httpd task's 12 kB stack, rather than inside
     * the Status that every /healthz also copies. */
    GpsTime::Sat sats[GpsTime::kMaxSats];
    const uint8_t have = GpsTime::instance().copySats(sats, GpsTime::kMaxSats);

    cJSON *list = cJSON_AddArrayToObject(root, "sats");
    if (list != nullptr) {
        for (uint8_t i = 0; i < have; ++i) {
            const GpsTime::Sat &s = sats[i];
            cJSON *row = cJSON_CreateObject();
            if (row == nullptr) {
                break;
            }
            cJSON_AddNumberToObject(row, "gnss", s.gnss);
            cJSON_AddNumberToObject(row, "sv", s.sv);
            cJSON_AddNumberToObject(row, "cno", s.cno);
            cJSON_AddNumberToObject(row, "elev", s.elev);
            cJSON_AddNumberToObject(row, "azim", s.azim);
            cJSON_AddNumberToObject(row, "flags", static_cast<double>(s.flags));
            /* Omitted rather than zeroed when the receiver has no residual: 0 cm
             * is a perfectly good residual, so the absence has to be its own
             * thing or every satellite looks like a perfect measurement. */
            if (s.prResValid) {
                cJSON_AddNumberToObject(row, "prres_cm", static_cast<double>(s.prRes));
            }
            cJSON_AddItemToArray(list, row);
        }
    }

    return sendJson(req, JsonWrapper(root), 200);
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

    /* Two independent jobs on one endpoint: the bias switch, and the lead's
     * delay. Either may be present. */
    std::string bias;
    const bool  haveBias = doc.GetField("bias", bias);
    double      delayNs  = 0;
    const bool  haveDelay = doc.GetField("delay_ns", delayNs);

    if (!haveBias && !haveDelay) {
        return sendJsonError(req, 400, "give \"bias\" (on|off) and/or \"delay_ns\"");
    }
    if (haveBias && bias != "on" && bias != "off") {
        return sendJsonError(req, 400, "bias must be \"on\" or \"off\"");
    }
    /* 200 us is more than 40 km of cable; below that the resolution of the
     * receiver's own solution is the limit, not the number. */
    if (haveDelay && (delayNs < 0 || delayNs > 200000)) {
        return sendJsonError(req, 400, "delay_ns must be 0..200000");
    }

    JsonWrapper resp;
    if (haveBias) {
        GpsTime::instance().requestAntennaBias(bias == "on");
        ESP_LOGW(TAG, "POST /gps/ant from %s: bias %s", callerAddress(req).c_str(), bias.c_str());
        resp.AddItem("requested", bias);
    }
    if (haveDelay) {
        /* Coax is about 5 ns per metre whatever its loss class (velocity factor
         * 0.66, and the spread across real cables is under 20%). The delay is
         * taken off the pulse, which is the same correction whether the extra
         * metres are in the lead or the antenna is simply further away. */
        /* setX() only changes the object; save() is what reaches NVS, and
         * without it the value is back to zero at the next boot and the pulse
         * reverts with it. */
        Settings::instance().setAntDelayNs(
            std::format("{}", static_cast<unsigned>(delayNs)));
        Settings::instance().save();
        GpsTime::instance().requestAntDelay();
        ESP_LOGW(TAG, "POST /gps/ant from %s: cable delay %u ns", callerAddress(req).c_str(),
                 static_cast<unsigned>(delayNs));
        resp.AddItem("delay_ns", delayNs);
    }

    /* Answered before it has happened: the GPS task owns the UART, so the
     * change is queued for its next pass. Say so, and say when to look. */
    resp.AddItem("read_back_after_s", 3.0);
    resp.AddItem("note", "drives GPIO17 for the bias; ant_bias, ant_short and ant_delay_ns at "
                         "/gps/info report the result");
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

esp_err_t NtpWebServer::gps_baud_post_handler(httpd_req_t *req)
{
    /* The host side alone, deliberately out of step with the receiver. The scan
     * puts the two back together within a few seconds, which is both the test
     * for that path and the way out of a line that has been desynchronised by
     * something that was not this firmware. */
    JsonWrapper doc;
    if (!readJsonBody(req, doc)) {
        return ESP_FAIL;
    }
    double baud = 0;
    if (!doc.GetField("baud", baud) || baud < 4800 || baud > 921600) {
        return sendJsonError(req, 400, "baud must be 4800..921600");
    }

    /* "module" moves the receiver too. Without it only the host moves and the
     * scan has to walk back, which is the safe way to recover from a line that
     * something else desynchronised. With it the receiver is written first and
     * the old rate is kept as a revert: if frames do not resume within two
     * silence windows they are restored, because otherwise the pair sits at
     * different rates until the scan happens to land on the right one. */
    bool moduleToo = false;
    doc.GetField("module", moduleToo);

    GpsTime::instance().requestBaud(static_cast<uint32_t>(baud), moduleToo);
    ESP_LOGW(TAG, "POST /gps/baud from %s: %u (module %s)", callerAddress(req).c_str(),
             static_cast<unsigned>(baud), moduleToo ? "yes" : "no");

    JsonWrapper resp;
    resp.AddItem("host_baud", baud);
    resp.AddItem("module", moduleToo);
    resp.AddItem("note", moduleToo
                             ? "receiver written too; reverts if frames do not resume in 10 s"
                             : "the receiver is untouched; gps_rescans at /healthz counts the "
                               "scan that puts the two back together");
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
        /* An array rather than a std::string: httpd_req_recv takes memory and
         * a length, and the reply is parsed from what it says it wrote. */
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

esp_err_t NtpWebServer::gps_survey_post_handler(httpd_req_t *req)
{
    /* Survey-in of the antenna position, and locking the result.
     *
     * A timing receiver does not want to be solving its own position at all: a
     * fixed position stops the clock from moving with the position solution,
     * and 1 m of position error is only 3.3 ns of time once it is averaged over
     * the constellation. That is what makes a survey worth running on a site
     * that will never be survey-grade.
     *
     * The accuracy limit is required rather than defaulted to something tight,
     * because the limit is what decides whether this ever finishes: the survey
     * ends only when both the minimum duration *and* the mean variance are met,
     * so a limit tighter than the sky can support runs until somebody gives up
     * -- and while it runs the position is not held, which is worse than the
     * moving mode it replaced. */
    JsonWrapper doc;
    if (!readJsonBody(req, doc)) {
        return ESP_FAIL;
    }

    std::string action;
    if (!doc.GetField("action", action)) {
        return sendJsonError(req, 400, "action must be \"start\", \"stop\" or \"fix\"");
    }

    if (action == "fix") {
        GpsTime::instance().requestFixPosition();
        ESP_LOGW(TAG, "POST /gps/survey from %s: lock the surveyed position",
                 callerAddress(req).c_str());
        JsonWrapper resp;
        resp.AddItem("action", "fix");
        resp.AddItem("note", "locks the receiver's own mean position; gps_time_mode and "
                             "gps_svin_* at /gps/info report the result");
        return sendJson(req, resp, 200);
    }

    if (action == "stop") {
        GpsTime::instance().requestSurvey(false);
        JsonWrapper resp;
        resp.AddItem("action", "stop");
        resp.AddItem("note", "back to a moving position; the last mean is kept for \"fix\"");
        return sendJson(req, resp, 200);
    }

    if (action != "start") {
        return sendJsonError(req, 400, "action must be \"start\", \"stop\" or \"fix\"");
    }

    /* Hours, because the useful values are in hours and nobody wants to say
     * 86400. Two hours is the shortest that means anything; a day is the
     * sensible default for a site that will stay put. */
    double hours = 24;
    doc.GetField("hours", hours);
    if (hours < 2 || hours > 48) {
        return sendJsonError(req, 400, "hours must be 2..48");
    }
    /* Default 30 m, not the 3 m that "survey-grade" suggests. The limit is
     * compared against the receiver's mean variance -- the *site's wander*,
     * measured here at about 26 m -- and not against how well the mean is known.
     * A limit under the wander does not make the result better, it only stops
     * the survey ever satisfying its own condition and leaves it running to the
     * duration with valid false. 30 m clears this site; 3 m never did. */
    double accMm = 30000;
    doc.GetField("acc_mm", accMm);
    if (accMm < 50 || accMm > 60000) {
        return sendJsonError(req, 400, "acc_mm must be 50..60000");
    }

    GpsTime::instance().requestSurvey(true, static_cast<uint32_t>(hours * 3600),
                                      static_cast<uint32_t>(accMm));
    ESP_LOGW(TAG, "POST /gps/survey from %s: start, %.1f h, %u mm limit",
             callerAddress(req).c_str(), hours, static_cast<unsigned>(accMm));

    JsonWrapper resp;
    resp.AddItem("action", "start");
    resp.AddItem("hours", hours);
    resp.AddItem("acc_mm", static_cast<double>(accMm));
    resp.AddItem("note", "gps_time_mode and gps_svin_* at /gps/info report progress; "
                         "lock it with {\"action\":\"fix\"} once it has finished");
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

    /* The same shape, and the only place in the image where the size of the
     * read is a design decision: esp_ota_write takes the chunk as it comes. */
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


