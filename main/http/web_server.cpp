#include "http/web_server.h"

#include "ptp/ptp.h"

#include <algorithm>
#include <cstdio>
#include <ctime>
#include <inttypes.h>
#include <optional>
#include <vector>
#include <string>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "clock/clock_source.h"
#include "app/config_apply.h"
#include "net/eth_link.h"
#include "gps/gps_time.h"
#include "gps/pps_latency.h"
#include "net/net_log.h"
#include "net/net_state.h"
#include "ntp/ntp_server.h"
#include "clock/ntp_time.h"
#include "ntp/serve_path.h"
#include "ntp/tx_lag.h"
#include "app/settings.h"
#include "app/text.h"

namespace {

constexpr const char *TAG = "webserver";

/* POST /ntp/sync blocks the request for at most this long. */
constexpr uint32_t kSyncWaitMs = 5000;

/* Long enough to watch holdover end and the reference demote and recover. */
constexpr int kMaxSuppressS = 3600;

/* 1 KB per chunk; the handler runs on the httpd task's stack. */
constexpr int kOtaChunkBytes = 1024;

/* IDF rejects a non-image at the first chunk (magic byte) or at esp_ota_end();
 * both get this message. */
constexpr const char *kNotAnImage = "not a valid app image for this chip";

/* WebServer needs a non-null context; the handlers use singletons. */
WebContext s_web_context;

const char *resetReasonText()
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_BROWNOUT:  return "brown-out";
    case ESP_RST_EXT:       return "external-pin";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "interrupt-watchdog";
    case ESP_RST_TASK_WDT:  return "task-watchdog";
    case ESP_RST_WDT:       return "other-watchdog";
    case ESP_RST_DEEPSLEEP: return "deep-sleep";
    case ESP_RST_SDIO:      return "sdio";
    default:                return "unknown";
    }
}

/* The clock reads 1970 until something sets it; there is no RTC. */
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

NtpWebServer::NtpWebServer() : WebServer(&s_web_context)
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
        esp_err_t (*handler)(httpd_req_t *req);
    };

    /* Kept in columns by hand. */
    // clang-format off
    static constexpr Route routes[] = {
        {"/config",       HTTP_GET,  config_get_handler},
        {"/config",       HTTP_POST, config_post_handler},
        {"/config/reset", HTTP_POST, config_reset_post_handler},
        {"/ntp/sync",     HTTP_POST, sync_post_handler},
        {"/ntp/txlag",    HTTP_POST, txlag_post_handler},
        {"/gps/pps",      HTTP_POST, gps_pps_post_handler},
        {"/gps/ant",      HTTP_POST, gps_ant_post_handler},
        {"/gps/gnss",     HTTP_POST, gps_gnss_post_handler},
        {"/gps/survey",   HTTP_POST, gps_survey_post_handler},
        {"/gps/info",     HTTP_GET,  gps_info_get_handler},
        {"/gps/bench",    HTTP_GET,  gps_bench_get_handler},
        {"/gps/sats",     HTTP_GET,  gps_sats_get_handler},
        {"/gps/samples",  HTTP_GET,  gps_samples_get_handler},
        {"/gps/reset",    HTTP_POST, gps_reset_post_handler},
        {"/gps/baud",     HTTP_POST, gps_baud_post_handler},
        {"/eth/restart",  HTTP_POST, eth_restart_post_handler},
        {"/firmware",     HTTP_GET,  firmware_get_handler},
        {"/firmware",     HTTP_POST, firmware_post_handler},
    };
    // clang-format on

    for (const Route &route : routes) {
        httpd_uri_t uri = {};
        uri.uri         = route.uri;
        uri.method      = route.method;
        uri.handler     = route.handler;
        uri.user_ctx    = webContext;

        esp_err_t err = httpd_register_uri_handler(server, &uri);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "register %s %s: %s", route.method == HTTP_POST ? "POST" : "GET",
                     route.uri, esp_err_to_name(err));
            return;
        }
    }
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
    json.AddItem("reset_reason", resetReasonText());
    json.AddItem("boots", std::strtoul(Settings::instance().boots().c_str(), nullptr, 10));
    json.AddItem("heap_free", static_cast<double>(esp_get_free_heap_size()));

    const Settings &settings = Settings::instance();
    const NtpTime  &ntp      = NtpTime::instance();

    json.AddItem("hostname", settings.hostname());
    json.AddItem("link_up", net_state::linkUp());
    json.AddItem("ip_mode", Settings::instance().ipMode());

    esp_netif_ip_info_t ip_info;
    esp_netif_t        *netif = net_state::netif();
    if (netif != nullptr && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK &&
        ip_info.ip.addr != 0) {
        const uint8_t *o = reinterpret_cast<const uint8_t *>(&ip_info.ip.addr);
        json.AddItem("ip", std::format("{}.{}.{}.{}", o[0], o[1], o[2], o[3]));
    }

#if CONFIG_LWIP_IPV6
    /* IPv6 addresses, each omitted until it exists (link-local after DAD, global
     * after a router advertisement). */
    if (const std::string link_local = net_state::ipv6LinkLocal(); !link_local.empty()) {
        json.AddItem("ipv6_linklocal", link_local);
    }
    if (const std::string global = net_state::ipv6Global(); !global.empty()) {
        json.AddItem("ipv6_global", global);
    }
#endif

    /* ntp_source is the configured setting, which can be "auto"; time_owner is
     * who has the clock and ntp_client what the SNTP client is doing. With the
     * receiver serving, ntp_synced means it is locked. */
    const ClockSource &arbiter = ClockSource::instance();
    json.AddItem("ntp_source", Settings::timeSourceName(settings.timeSource()));
    json.AddItem("ntp_client", ntp.sourceName());
    json.AddItem("time_owner", ClockSource::name(arbiter.owner()));
    json.AddItem("gps_fallback", arbiter.owner() != ClockOwner::Gps);
    json.AddItem("fallback_count", static_cast<double>(arbiter.engagements()));
    if (arbiter.owner() != ClockOwner::Gps) {
        json.AddItem("fallback_age_s", static_cast<double>(arbiter.ageS()));
    }

    /* The receiver's headline state, reported whatever is serving. */
    const GpsTime::Status gps = GpsTime::instance().status();
    json.AddItem("gps_fix", static_cast<double>(gps.fixType));
    json.AddItem("gps_sats", static_cast<double>(gps.satellites));
    if (gps.ppsAgeUs >= 0) {
        json.AddItem("gps_pps_age_ms", static_cast<double>(gps.ppsAgeUs / 1000));
    }

    if (arbiter.owner() != ClockOwner::Upstream) {
        json.AddNull("ntp_server");
        json.AddItem("ntp_synced", gps.locked);
        if (gps.lastFix != 0) {
            json.AddItem("last_sync", static_cast<double>(gps.lastFix));
        }
        /* Time validity and the edge count, which `locked` folds together. */
        json.AddItem("gps_time_valid", gps.timeValid);
        json.AddItem("gps_pps_count", static_cast<double>(gps.ppsCount));
        json.AddItem("gps_anchored", gps.anchored);
        /* Framing: good frames, checksum or length drops, baud rescans. */
        json.AddItem("gps_frames", static_cast<double>(gps.frames));
        json.AddItem("gps_dropped_crc", static_cast<double>(gps.droppedChecksum));
        json.AddItem("gps_dropped_len", static_cast<double>(gps.droppedLength));
        json.AddItem("gps_rescans", static_cast<double>(gps.rescans));
        /* Only whether it is working; the detail is at /gps/info. The configuration
         * failures stay, since they make the rest meaningless. */
        if (gps.tp5Rejected) {
            json.AddItem("gps_tp5_rejected", true);
        }
        if (gps.antCfgRejected) {
            json.AddItem("gps_ant_cfg_rejected", true);
        }
        if (gps.antShort) {
            json.AddItem("gps_ant_short", true);
        }
        if (gps.antLatched) {
            json.AddItem("gps_ant_latched", true);
        }
        if (gps.ppsSuppressUs > 0) {
            json.AddItem("gps_pps_suppressed_s", static_cast<double>(gps.ppsSuppressUs / 1000000));
        }
        /* The last holdover's length and the prediction error when the edge came
         * back; absent until there has been one. */
        if (gps.lastHoldoverUs > 0) {
            json.AddItem("gps_last_holdover_s", static_cast<double>(gps.lastHoldoverUs) / 1e6);
            json.AddItem("gps_last_holdover_error_us",
                         static_cast<double>(gps.lastHoldoverErrNs) / 1000.0);
        }
    } else {
        if (const std::optional<std::string> upstream = ntp.upstreamName()) {
            json.AddItem("ntp_server", *upstream);
            json.AddItem("ntp_reachable", ntp.upstreamReachable());
        } else {
            json.AddNull("ntp_server");
        }
        /* A recent sync, not merely one at some point. */
        json.AddItem("ntp_synced", arbiter.upstreamLocked());
        if (ntp.lastSync() != 0) {
            json.AddItem("last_sync", static_cast<double>(ntp.lastSync()));
        }
    }

    const NtpServer &server = NtpServer::instance();
    json.AddItem("serve_ntp", server.running());
    /* Hardware timestamp counters; zero means software stamps. */
    const ptp::Stats stamps = ptp::stats();
    json.AddItem("phc", ptp::hardware());
    json.AddItem("hw_rx_stamps", static_cast<double>(stamps.rxStamps));
    json.AddItem("hw_tx_stamps", static_cast<double>(stamps.txStamps));
    json.AddItem("hw_tx_missing", static_cast<double>(stamps.txMissing));
    json.AddItem("hw_tx_errors", static_cast<double>(stamps.txErrors));
    json.AddItem("ptp_mode", ptp::modeText(ptp::mode()));
    if (ptp::mode() != ptp::Mode::Off) {
        json.AddItem("ptp_domain", static_cast<double>(Settings::instance().ptpDomain()));
        json.AddItem("ptp_sync_sent", static_cast<double>(stamps.syncSent));
        json.AddItem("ptp_announce_sent", static_cast<double>(stamps.announceSent));
        json.AddItem("ptp_delay_resp", static_cast<double>(stamps.delayResp));
        if (ptp::mode() == ptp::Mode::Slave) {
            json.AddItem("ptp_offset_ns", static_cast<double>(stamps.slaveOffsetNs));
            json.AddItem("ptp_slave_measured", static_cast<double>(stamps.measured));
        }
        json.AddItem("ptp_frames", static_cast<double>(stamps.frames));
        if (stamps.lastPeerNs != 0) {
            char peer[24] = {};
            std::snprintf(peer, sizeof(peer), "%02x:%02x:%02x:%02x:%02x:%02x", stamps.lastPeer[0],
                          stamps.lastPeer[1], stamps.lastPeer[2], stamps.lastPeer[3],
                          stamps.lastPeer[4], stamps.lastPeer[5]);
            json.AddItem("ptp_last_peer", peer);
        }
    }
    if (server.running()) {
        const TimeReference ref = server.reference();
        json.AddItem("ntp_stratum", static_cast<double>(ref.stratum));
        json.AddItem("ntp_refid", refIdText(ref.refId, ref.stratum));
        /* Leap indicator a reply would carry now: 1 or 2 before a leap, 3 when not
         * locked, else 0. */
        json.AddItem("ntp_leap", static_cast<double>(ref.locked ? ref.leap : 3));
        /* Locked but over the dispersion gate. */
        json.AddItem("ntp_demoted", ref.demoted);
        /* Root delay and dispersion a reply would carry, before 16.16 rounding. For
         * the GPS the delay is 0 and dispersion is its error budget; for the upstream
         * both start at about 1 ms and dispersion grows with age. */
        const NtpServer::RootFigures root = server.wireRootFigures();
        json.AddItem("ntp_root_delay_us", root.delayUs);
        json.AddItem("ntp_root_dispersion_us", root.dispersionUs);
        json.AddItem("ntp_requests", static_cast<double>(server.requests()));
        const NtpServer::Counters counters = server.counters();
        json.AddItem("ntp_answered", static_cast<double>(counters.answered));
        json.AddItem("ntp_kod", static_cast<double>(counters.kod));
        json.AddItem("ntp_dropped", static_cast<double>(counters.dropped));
        json.AddItem("ntp_rate_per_min", counters.perMinute);
        /* TxLag, and how many replies used it (only the sendto() path does). */
        json.AddItem("ntp_tx_correction_us", server.txLagUs());
        json.AddItem("ntp_tx_corrected", static_cast<double>(server.txCorrected()));
        json.AddItem("ntp_tx_scatter_us", server.txScatterUs());
        /* The ServePath term in dispersion. */
        json.AddItem("ntp_il_try", static_cast<double>(server.interleavedTried()));
        json.AddItem("ntp_il_saved", static_cast<double>(server.interleavedSaved()));
        json.AddItem("ntp_il_keyonly", static_cast<double>(server.interleavedKeyOnly()));
        json.AddItem("ntp_il_hit", static_cast<double>(server.interleavedHit()));
        json.AddItem("ntp_serve_bound_us", ServePath::instance().boundUs());
        json.AddItem("ntp_interleaved_served", static_cast<double>(server.interleavedServed()));
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
    /* Reads a JSON object body (4 KB cap), sending the error reply itself. */
    JsonWrapper doc;
    if (!readJsonBodyTimed(req, doc)) {
        return ESP_FAIL;
    }

    const std::string caller = callerAddress(req);

    Settings         &settings        = Settings::instance();
    const std::string hostname_before = settings.hostname();

    /* Only write NVS when something changed. */
    const std::string before = settings.toJson().ToString();
    settings.applyJson(doc);

    /* Here rather than in an onChange hook, which also runs at boot before lwIP. */
    const bool logChanged = doc.ContainsField("logHost") || doc.ContainsField("logPort");
    if (settings.toJson().ToString() != before) {
        settings.save();
    }
    settings.log();

    /* Shared with the console, so changes apply now and in the same order. */
    config_apply::afterChange(hostname_before, logChanged);
    ESP_LOGI(TAG, "POST /config from %s", caller.c_str());

    return sendJson(req, settings.toJson(), 200);
}

esp_err_t NtpWebServer::config_reset_post_handler(httpd_req_t *req)
{
    const std::string caller = callerAddress(req);

    Settings         &settings        = Settings::instance();
    const std::string hostname_before = settings.hostname();
    /* Defaults leave the boot count alone. */
    const std::string boots = settings.boots();
    settings.resetToDefaults(); /* fires the tz change callback */
    settings.setBoots(boots);
    settings.save();
    settings.log();
    config_apply::afterChange(hostname_before, true);
    ESP_LOGW(TAG, "POST /config/reset from %s: back to compiled-in defaults", caller.c_str());

    JsonWrapper resp;
    resp.AddItem("status", "reset");
    return sendJson(req, resp, 200);
}

/* POST /ntp/txlag {"enabled":true|false}: switch the sendto() path's transmit
 * correction off, to measure what it is worth. */
esp_err_t NtpWebServer::txlag_post_handler(httpd_req_t *req)
{
    JsonWrapper doc;
    if (!readJsonBodyTimed(req, doc)) {
        return ESP_FAIL;
    }

    bool enabled = true;
    if (!doc.GetField("enabled", enabled)) {
        return sendJsonError(req, 400, "expected {\"enabled\": true|false}");
    }

    TxLag &lag = TxLag::instance();
    lag.setEnabled(enabled);
    ESP_LOGW(TAG, "POST /ntp/txlag from %s: transmit correction %s", callerAddress(req).c_str(),
             enabled ? "on" : "off");

    JsonWrapper resp;
    resp.AddItem("enabled", lag.enabled());
    return sendJson(req, resp, 200);
}

esp_err_t NtpWebServer::sync_post_handler(httpd_req_t *req)
{
    const std::string caller   = callerAddress(req);
    const Settings   &settings = Settings::instance();

    /* The GPS is not polled, so while it is serving this reports its state
     * instead of forcing a poll. off/dhcp/static always use the client below. */
    const Settings::TimeSource configured = settings.timeSource();
    const bool                 gpsServing = configured == Settings::TimeSource::Gps ||
                            (configured == Settings::TimeSource::Auto &&
                             ClockSource::instance().owner() != ClockOwner::Upstream);
    if (gpsServing) {
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
    /* Holdover test: the PPS line is soldered on, so this makes the interrupt
     * ignore edges for a while. /healthz reports the time left. */
    JsonWrapper doc;
    if (!readJsonBodyTimed(req, doc)) {
        return ESP_FAIL;
    }

    int seconds = 0;
    if (!doc.GetField("suppress_s", seconds) || seconds < 0 || seconds > kMaxSuppressS) {
        return sendJsonError(req, 400, "suppress_s must be 0..3600, or 0 to end the test");
    }

    const std::string     caller  = callerAddress(req);
    const uint32_t        applied = GpsTime::instance().suppressPps(static_cast<uint32_t>(seconds));
    const GpsTime::Status gps     = GpsTime::instance().status();

    JsonWrapper resp;
    resp.AddItem("pps_suppressed_s", static_cast<double>(applied));
    resp.AddItem("locked", gps.locked);
    resp.AddItem("stratum", static_cast<double>(gps.locked ? 1 : 16));
    resp.AddItem("holdover_s", static_cast<double>(gps.ppsAgeUs > 0 ? gps.ppsAgeUs / 1000000 : 0));
    ESP_LOGW(TAG, "POST /gps/pps from %s: suppress_s=%u", caller.c_str(),
             static_cast<unsigned>(applied));
    return sendJson(req, resp, 200);
}

esp_err_t NtpWebServer::gps_samples_get_handler(httpd_req_t *req)
{
    /* GET /gps/samples?since=<seq>&max=<n>. since=0 starts from the oldest record.
     * `lost` is how many records fell out of the ring before the cursor. */
    uint32_t since = 0;
    size_t   max   = 120;
    char     buf[96];
    if (httpd_req_get_url_query_len(req) > 0 &&
        httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char value[16];
        if (httpd_query_key_value(buf, "since", value, sizeof(value)) == ESP_OK) {
            since = static_cast<uint32_t>(std::strtoul(value, nullptr, 10));
        }
        if (httpd_query_key_value(buf, "max", value, sizeof(value)) == ESP_OK) {
            const unsigned long asked = std::strtoul(value, nullptr, 10);
            if (asked > 0 && asked < 400) {
                max = asked;
            }
        }
    }

    static GpsTime::Sample page[400];
    uint32_t               first = 0;
    uint32_t               last  = 0;
    uint32_t               lost  = 0;
    const size_t got = GpsTime::instance().drainSamples(since, page, max, first, last, lost);

    /* Arrays of nine fields, built by hand: much smaller than named objects. The
     * ninth is the dispersion stated for that second. */
    std::string body;
    body.reserve(got * 82 + 128);
    body += std::format("{{\"firstSeq\":{},\"lastSeq\":{},\"lost\":{},\"samples\":[", first, last,
                        lost);
    for (size_t i = 0; i < got; ++i) {
        const GpsTime::Sample &s = page[i];
        if (i != 0) {
            body += ',';
        }
        body += std::format("[{},{},{},{},{},{},{},{},{}]", s.seq, s.epoch,
                            s.kind == 0 ? s.offsetNs : 0, s.ratePpb, s.jitterNs, s.flags, s.sats,
                            s.kind, s.dispersionNs);
    }
    body += "]}";

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "cache-control", "no-store");
    return httpd_resp_send(req, body.c_str(), body.size());
}

/* The receiver diagnostics. /gps/info sends the keys in kPolled; /gps/bench
 * sends everything. */
static void emitGpsInfo(const GpsTime::Status &gps, JsonWrapper &doc)
{
    /* Who owns the clock, which the offsets below are measured against. */
    const ClockSource &arbiter = ClockSource::instance();
    doc.AddItem("time_owner", ClockSource::name(arbiter.owner()));
    doc.AddItem("gps_fallback", arbiter.owner() != ClockOwner::Gps);

    if (!gps.moduleId.empty()) {
        doc.AddItem("module", gps.moduleId);
    }
    doc.AddItem("ant_cfg", gps.antCfgRejected ? "rejected"
                           : gps.antCfgAcked  ? "accepted"
                                              : "silent");
    doc.AddItem("tp5_ok", !gps.tp5Rejected);
    doc.AddItem("timtp", gps.timTpRejected ? "rejected" : gps.timTpAcked ? "accepted" : "silent");

    doc.AddItem("offset_us", static_cast<double>(gps.offsetUs));
    doc.AddItem("offset_ns", static_cast<double>(gps.offsetNs));
    doc.AddItem("pps_ptp_ns", static_cast<double>(gps.ppsPtpNs));
    doc.AddItem("phc_mono_delta_ns", static_cast<double>(gps.phcMonoDeltaNs));
    doc.AddItem("phc_phase_ns", static_cast<double>(gps.phcPhaseNs));
    doc.AddItem("phc_steps", static_cast<double>(gps.phcSteps));
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
        doc.AddItem("svin_mean_x_cm", static_cast<double>(gps.svinMeanCm[0]));
        doc.AddItem("svin_mean_y_cm", static_cast<double>(gps.svinMeanCm[1]));
        doc.AddItem("svin_mean_z_cm", static_cast<double>(gps.svinMeanCm[2]));
        doc.AddItem("svin_target_s", gps.svinTargetS);
        doc.AddItem("svin_limit_mm", gps.svinLimitMm);
        doc.AddItem("svin_valid", gps.svinValid);
        doc.AddItem("svin_age_s", gps.svinAgeS);
        doc.AddItem("hold_validated", gps.holdValidated);
    }
    if (gps.locked) {
        doc.AddItem("dispersion_us", static_cast<double>(gps.dispersionNs) / 1000.0);
        doc.AddItem("lock_window_s", static_cast<double>(gps.lockWindowUs) / 1000000);
        /* Present only while the pulse position is unobserved. */
        if (gps.pulseUnobservedNs > 0) {
            doc.AddItem("pulse_unobserved_us", static_cast<double>(gps.pulseUnobservedNs) / 1000.0);
        }
    }
    /* Our stamping jitter from the rate fit; does not need TIM-TP. */
    doc.AddItem("tp_jitter_ns", static_cast<double>(gps.jitterNs));
    /* Edge-to-frame delay, against kPairWindowUs; the maximum is the one to read. */
    if (gps.pairAgeS >= 0) {
        doc.AddItem("pair_age_us", static_cast<double>(gps.pairAgeUs));
        doc.AddItem("pair_age_max_us", static_cast<double>(gps.pairAgeMaxUs));
        doc.AddItem("pair_age_s", gps.pairAgeS);
    }
    if (gps.tosCfg != 0) {
        doc.AddItem("tos_cfg", gps.tosCfg == 1 ? "accepted" : "rejected");
    }
    if (gps.tosFlags != 0) {
        doc.AddItem("tos_locked", (gps.tosFlags & (1u << 13)) != 0);
        doc.AddItem("tos_coherent", (gps.tosFlags & (1u << 12)) != 0);
        doc.AddItem("tos_utc_offset_ns", static_cast<double>(gps.tosUtcOffsetNs));
        doc.AddItem("tos_utc_unc_ns", static_cast<double>(gps.tosUtcUncNs));
    }
    doc.AddItem("tp5_cable_ns", static_cast<double>(gps.tp5CableNs));
    doc.AddItem("tp5_user_ns", static_cast<double>(gps.tp5UserNs));
    doc.AddItem("tp5_flags", static_cast<double>(gps.tp5Flags));
    doc.AddItem("tp5_period_us", static_cast<double>(gps.tp5PeriodUs));
    doc.AddItem("tp5_ratio", static_cast<double>(gps.tp5Ratio));
    if (gps.rateCfg != 0) {
        doc.AddItem("rate_cfg", gps.rateCfg == 1 ? "accepted" : "rejected");
    }
    if (gps.nav5Cfg != 0) {
        doc.AddItem("nav5_cfg", gps.nav5Cfg == 1 ? "accepted" : "rejected");
    }
    doc.AddItem("timtp_tim_replies", static_cast<double>(gps.timTpTimN));
    doc.AddItem("timtp_nav_replies", static_cast<double>(gps.timTpNavN));
    if (gps.timTpCount != 0) {
        if (gps.qErrValid) {
            doc.AddItem("tp_qerr_ns", static_cast<double>(gps.qErrNs));
        }
        doc.AddItem("tp_on_second", gps.pulseOnSecond);
        /* The pulse's offset in its second, per TIM-TP. */
        doc.AddItem("tp_offset_ns", static_cast<double>(gps.pulseOffsetNs));
        /* The same epoch's milliseconds on the GPS time base; should agree with
         * pvt_nano_ns, since GPS and UTC differ by whole seconds. */
        doc.AddItem("pvt_itow_ms", static_cast<double>(gps.pvtItowMs % 1000));
        /* bit 0 time base (0 GNSS, 1 UTC), bit 1 UTC available, bit 4 qErr invalid. */
        doc.AddItem("timtp_flags", static_cast<double>(gps.timTpFlags));
    }

    /* Servo counters and root delay (0). */
    doc.AddItem("root_delay_ns", static_cast<double>(gps.rootDelayNs));
    doc.AddItem("servo_steps", static_cast<double>(gps.servoSteps));
    doc.AddItem("servo_clamps", static_cast<double>(gps.servoClamps));
    doc.AddItem("edge_spikes", static_cast<double>(gps.edgeSpikes));
    doc.AddItem("edge_outliers", static_cast<double>(gps.edgeOutliers));
    doc.AddItem("status_seq", static_cast<double>(gps.snapshotSeq));
    doc.AddItem("status_age_ms", static_cast<double>(gps.snapshotAgeUs) / 1000.0);
    /* Refused edges, and whether the newest passed. */
    doc.AddItem("edge_rejects", static_cast<double>(gps.edgeRejects));
    doc.AddItem("last_edge_trusted", gps.lastEdgeTrusted);
    doc.AddItem("tie_max_ns", gps.tieMaxNs);
    doc.AddItem("tie_rms_ns", gps.tieRmsNs);
    /* PPS interrupt latency from the GPTimer capture; min is the fixed path. */
    pps_latency::Stats latency = {};
    if (pps_latency::stats(&latency)) {
        doc.AddItem("pps_latency_last_ns", static_cast<double>(latency.lastNs));
        doc.AddItem("pps_latency_min_ns", static_cast<double>(latency.minNs));
        doc.AddItem("pps_latency_max_ns", static_cast<double>(latency.maxNs));
        doc.AddItem("pps_latency_edges", static_cast<double>(latency.edges));
        doc.AddItem("pps_latency_unmatched", static_cast<double>(latency.unmatched));
        doc.AddItem("pps_latency_dropped", static_cast<double>(latency.dropped));
        doc.AddItem("pps_latency_hist",
                    std::format("{}/{}/{}/{}/{}/{}", latency.hist[0], latency.hist[1],
                                latency.hist[2], latency.hist[3], latency.hist[4],
                                latency.hist[5]));
    }
    /* NAV-PVT's offset into its second. */
    doc.AddItem("pvt_nano_ns", gps.pvtNanoNs);
    doc.AddItem("pulse_offset_ns", gps.pulseOffsetNs);
    doc.AddItem("pulse_on_second", gps.pulseOnSecond);
    doc.AddItem("hold_wanted", gps.holdWanted);
    doc.AddItem("hold_assert_in_s", gps.holdAssertInS);
    doc.AddItem("hold_ready", gps.holdReady);
    doc.AddItem("last_edge_age_ms", gps.lastEdgeAgeMs);

    /* Each with its age, so a message that stops shows as a growing age. */
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
    /* UART link health. Byte counters are cumulative. */
    /* Not gated on any message having arrived. */
    doc.AddItem("baud", static_cast<double>(gps.baud));
    doc.AddItem("ticks_per_us", static_cast<double>(gps.ticksPerUs));
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

    /* Reported even when the message is refused. */
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
    /* C/N0 for the antenna; AGC, noise and jamming for front-end overload. */
    if (gps.monHwValid) {
        static constexpr const char *kAntStatus[] = {"init", "unknown", "ok", "short", "open"};
        static constexpr const char *kAntPower[]  = {"off", "on", "unknown"};
        static constexpr const char *kJam[]       = {"unknown", "ok", "warning", "critical"};
        doc.AddItem("ant", gps.antStatus <= 4 ? kAntStatus[gps.antStatus] : "unknown");
        doc.AddItem("ant_power", gps.antPower <= 2 ? kAntPower[gps.antPower] : "unknown");
        doc.AddItem("ant_bias", gps.antBiasOn ? "on" : "off");
        doc.AddItem("ant_short", gps.antShort);
        doc.AddItem("ant_delay_ns", gps.antDelayNs);
        doc.AddItem("user_delay_ns", gps.userDelayNs);
        doc.AddItem("noise_dbm", -static_cast<double>(gps.noisePerMs));
        doc.AddItem("agc", static_cast<double>(gps.agcCnt));
        doc.AddItem("jam_ind", static_cast<double>(gps.jamInd));
        doc.AddItem("jam", gps.jamState <= 3 ? kJam[gps.jamState] : "unknown");
    }
    doc.AddItem("cn0_max", static_cast<double>(gps.cn0Max));
    doc.AddItem("cn0_mean", static_cast<double>(gps.cn0Mean));
    doc.AddItem("cn0_used", static_cast<double>(gps.cn0Used));
    if (gps.leapValid) {
        doc.AddItem("leap_s", static_cast<double>(gps.leapSeconds));
        const bool pending = gps.leapChange != 0 && gps.leapInS > 0;
        doc.AddItem("leap_pending", pending);
        if (pending) {
            doc.AddItem("leap_in_s", static_cast<double>(gps.leapInS));
        }
    }
    if (gps.lastHoldoverUs > 0) {
        /* The previous outage, not the current edge age. */
        doc.AddItem("last_holdover_s", static_cast<double>(gps.lastHoldoverUs) / 1e6);
        doc.AddItem("last_holdover_error_us", static_cast<double>(gps.lastHoldoverErrNs) / 1000.0);
    }

    if (gps.monHwValid) {
        /* Bit maps, as hex. */
        doc.AddItem("pin_sel", std::format("{:08x}", gps.pinSel));
        doc.AddItem("pin_dir", std::format("{:08x}", gps.pinDir));
        doc.AddItem("pin_val", std::format("{:08x}", gps.pinVal));
        doc.AddItem("mon_hw_age_s", static_cast<double>(gps.monHwAgeUs) / 1e6);
    }
    if (gps.navSatAgeUs >= 0) {
        doc.AddItem("cn0_age_s", static_cast<double>(gps.navSatAgeUs) / 1e6);
    }
}

/* Keys /gps/info keeps; everything else is only in /gps/bench. */
static constexpr const char *kPolled[] = {
    "module",
    "tp5_ok",
    "timtp",
    "offset_ns",
    "pps_ptp_ns",
    "phc_mono_delta_ns",
    "phc_phase_ns",
    "rate_ppm",
    "rate_n",
    "bridge_active",
    "bridge_clients",
    "gnss",
    "gnss_cfg",
    "time_mode",
    "svin_dur_s",
    "svin_acc_mm",
    "svin_var_mm2",
    "svin_obs",
    "svin_target_s",
    "svin_limit_mm",
    "svin_valid",
    "dispersion_us",
    "lock_window_s",
    "tp_jitter_ns",
    "root_delay_ns",
    "servo_steps",
    "servo_clamps",
    "edge_spikes",
    "edge_outliers",
    "status_seq",
    "status_age_ms",
    "tie_max_ns",
    "tie_rms_ns",
    "pps_latency_max_ns",
    "pvt_nano_ns",
    "pulse_offset_ns",
    "pulse_on_second",
    "hold_wanted",
    "hold_assert_in_s",
    "hold_ready",
    "last_edge_age_ms",
    "gdop",
    "pdop",
    "hdop",
    "vdop",
    "dop_age_s",
    "clk_bias_ns",
    "clk_drift_ns_s",
    "t_acc_ns",
    "f_acc_ps_s",
    "nav_status_flags",
    "nav_status_flags2",
    "ttff_ms",
    "baud",
    "recent_frames",
    "io_rx_bytes",
    "io_tx_bytes",
    "io_parity",
    "io_framing",
    "io_overrun",
    "rbuf_pending",
    "rbuf_peak",
    "tbuf_pending",
    "tbuf_peak",
    "tbuf_errors",
    "msg_ubx",
    "msg_skipped",
    "msgpp_hex",
    "io_hex",
    "rf_cfg",
    "rf_bands",
    "rf_jam_worst",
    "rf_agc_min",
    "ant",
    "ant_power",
    "ant_bias",
    "ant_short",
    "ant_delay_ns",
    "noise_dbm",
    "agc",
    "jam_ind",
    "jam",
    "cn0_max",
    "cn0_mean",
    "cn0_used",
    "leap_pending",
    "leap_in_s",
    "last_holdover_s",
    "last_holdover_error_us",
    "edge_rejects",
    "last_edge_trusted",
    "pair_age_us",
    "pair_age_max_us",
    "pair_age_s",
    "time_owner",
    "gps_fallback",
};

/* Remove keys not in kPolled. Names are collected first, since deleting an
 * item invalidates the iteration. */
static void stripToPolled(cJSON *obj)
{
    std::vector<std::string> drop;
    for (const cJSON *item = obj->child; item != nullptr; item = item->next) {
        bool keep = false;
        for (const char *name : kPolled) {
            if (item->string != nullptr && std::strcmp(item->string, name) == 0) {
                keep = true;
                break;
            }
        }
        if (!keep && item->string != nullptr) {
            drop.emplace_back(item->string);
        }
    }
    for (const std::string &name : drop) {
        cJSON_DeleteItemFromObjectCaseSensitive(obj, name.c_str());
    }
}

esp_err_t NtpWebServer::gps_info_get_handler(httpd_req_t *req)
{
    const GpsTime::Status gps = GpsTime::instance().status();
    JsonWrapper           doc;
    emitGpsInfo(gps, doc);

    /* Strip the tree in place rather than print and reparse. */
    cJSON *obj = doc.Release();
    if (obj == nullptr) {
        return sendJsonError(req, 500, "out of memory");
    }
    stripToPolled(obj);
    char *text = cJSON_PrintUnformatted(obj);
    cJSON_Delete(obj);
    if (text == nullptr) {
        return sendJsonError(req, 500, "out of memory");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "cache-control", "no-store");
    const esp_err_t err = httpd_resp_send(req, text, HTTPD_RESP_USE_STRLEN);
    free(text);
    return err;
}

esp_err_t NtpWebServer::gps_bench_get_handler(httpd_req_t *req)
{
    /* Everything, for a bench session or a bug report. */
    const GpsTime::Status gps = GpsTime::instance().status();
    JsonWrapper           doc;
    emitGpsInfo(gps, doc);
    return sendJson(req, doc, 200);
}

esp_err_t NtpWebServer::gps_sats_get_handler(httpd_req_t *req)
{
    /* Per-satellite C/N0, elevation and azimuth for the sky view, separate from
     * /gps/info because its size follows the sky. flags is the receiver's word:
     * qualityInd bits 0-2, svUsed 3, health 4-5, orbitSource 8-10, ephAvail 11. */
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

    /* On the httpd task's 12 KB stack, not in the Status copied by every call. */
    GpsTime::Sat  sats[GpsTime::kMaxSats];
    const uint8_t have = GpsTime::instance().copySats(sats, GpsTime::kMaxSats);

    cJSON *list = cJSON_AddArrayToObject(root, "sats");
    if (list != nullptr) {
        for (uint8_t i = 0; i < have; ++i) {
            const GpsTime::Sat &s   = sats[i];
            cJSON              *row = cJSON_CreateObject();
            if (row == nullptr) {
                break;
            }
            cJSON_AddNumberToObject(row, "gnss", s.gnss);
            cJSON_AddNumberToObject(row, "sv", s.sv);
            cJSON_AddNumberToObject(row, "cno", s.cno);
            cJSON_AddNumberToObject(row, "elev", s.elev);
            cJSON_AddNumberToObject(row, "azim", s.azim);
            cJSON_AddNumberToObject(row, "flags", static_cast<double>(s.flags));
            /* Omitted when not available, since 0 is a valid residual. */
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
    /* Antenna bench settings: bias, cable delay, user delay, pulse width and
     * period. */
    JsonWrapper doc;
    if (!readJsonBodyTimed(req, doc)) {
        return ESP_FAIL;
    }

    std::string bias;
    const bool  haveBias   = doc.GetField("bias", bias);
    double      delayNs    = 0;
    const bool  haveDelay  = doc.GetField("delay_ns", delayNs);
    double      pulseMs    = 0;
    const bool  havePulse  = doc.GetField("pulse_ms", pulseMs);
    double      userNs     = 0;
    const bool  haveUser   = doc.GetField("user_ns", userNs);
    double      periodUs   = 0;
    const bool  havePeriod = doc.GetField("period_us", periodUs);

    if (!haveBias && !haveDelay && !havePulse && !haveUser && !havePeriod) {
        return sendJsonError(req, 400, "give \"bias\" (on|off), \"delay_ns\" and/or \"pulse_ms\"");
    }
    if (haveBias && bias != "on" && bias != "off") {
        return sendJsonError(req, 400, "bias must be \"on\" or \"off\"");
    }
    /* 200 us is over 40 km of coax. */
    if (haveDelay && (delayNs < 0 || delayNs > 200000)) {
        return sendJsonError(req, 400, "delay_ns must be 0..200000");
    }
    if (havePulse && (pulseMs < 1 || pulseMs > 500)) {
        return sendJsonError(req, 400, "pulse_ms must be 1..500");
    }
    /* Period 0.1 to 10 s; user delay up to +/-10 ms. */
    if (havePeriod && (periodUs < 100000 || periodUs > 10000000)) {
        return sendJsonError(req, 400, "period_us must be 100000..10000000");
    }
    if (haveUser && (userNs < -10000000 || userNs > 10000000)) {
        return sendJsonError(req, 400, "user_ns must be -10000000..10000000");
    }

    JsonWrapper resp;
    if (haveBias) {
        GpsTime::instance().requestAntennaBias(bias == "on");
        ESP_LOGW(TAG, "POST /gps/ant from %s: bias %s", callerAddress(req).c_str(), bias.c_str());
        resp.AddItem("requested", bias);
    }
    if (haveDelay) {
        /* Coax is about 5 ns per metre. */
        /* save() writes NVS; set alone is lost at reboot. */
        Settings::instance().setAntDelayNs(std::format("{}", static_cast<unsigned>(delayNs)));
        Settings::instance().save();
        GpsTime::instance().requestAntDelay();
        ESP_LOGW(TAG, "POST /gps/ant from %s: cable delay %u ns", callerAddress(req).c_str(),
                 static_cast<unsigned>(delayNs));
        resp.AddItem("delay_ns", delayNs);
    }

    if (haveUser) {
        Settings::instance().setGpsUserDelayNs(std::format("{}", static_cast<long>(userNs)));
        Settings::instance().save();
        GpsTime::instance().requestAntDelay();
        ESP_LOGW(TAG, "POST /gps/ant from %s: user delay %ld ns", callerAddress(req).c_str(),
                 static_cast<long>(userNs));
        resp.AddItem("user_ns", userNs);
    }
    if (havePeriod) {
        GpsTime::instance().requestPeriodUs(static_cast<uint32_t>(periodUs));
        ESP_LOGW(TAG, "POST /gps/ant from %s: period %u us", callerAddress(req).c_str(),
                 static_cast<unsigned>(periodUs));
        resp.AddItem("period_us", periodUs);
    }
    if (havePulse) {
        GpsTime::instance().requestPulseMs(static_cast<uint32_t>(pulseMs));
        ESP_LOGW(TAG, "POST /gps/ant from %s: pulse width %u ms", callerAddress(req).c_str(),
                 static_cast<unsigned>(pulseMs));
        resp.AddItem("pulse_ms", pulseMs);
    }

    /* Queued for the GPS task, which owns the UART. */
    resp.AddItem("read_back_after_s", 3.0);
    resp.AddItem("note", "drives GPIO17 for the bias; ant_bias, ant_short and ant_delay_ns at "
                         "/gps/info report the result");
    return sendJson(req, resp, 200);
}

esp_err_t NtpWebServer::gps_gnss_post_handler(httpd_req_t *req)
{
    /* Constellations. This receiver NAKs BeiDou with GLONASS or Galileo, so the
     * named sets are "max" (GPS, Galileo, GLONASS, QZSS) and "beidou" (GPS,
     * BeiDou, QZSS). A raw mask is accepted too, and a refusal is reported. */
    JsonWrapper doc;
    if (!readJsonBodyTimed(req, doc)) {
        return ESP_FAIL;
    }

    /* SBAS and IMES are off in both sets. */
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
            return sendJsonError(req, 400,
                                 "need {\"set\":\"max\"|\"beidou\"} or {\"mask\":1..127}");
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
    /* Host side only: the scan finds the receiver again, which tests that path. */
    JsonWrapper doc;
    if (!readJsonBodyTimed(req, doc)) {
        return ESP_FAIL;
    }
    double baud = 0;
    if (!doc.GetField("baud", baud) || baud < 4800 || baud > 921600) {
        return sendJsonError(req, 400, "baud must be 4800..921600");
    }

    /* "module" changes the receiver first, then the host, and restores the old
     * rate if frames do not resume within four silence windows. */
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
    /* Hot start by default: keeps the almanac. */
    bool cold     = false;
    bool hard     = false;
    bool power    = false;
    bool defaults = false;
    /* A body that is present must parse. */
    if (req->content_len > 0) {
        JsonWrapper doc;
        if (!readJsonBodyTimed(req, doc)) {
            return ESP_FAIL;
        }
        doc.GetField("cold", cold);
        doc.GetField("hardware", hard);
        doc.GetField("power", power);
        doc.GetField("defaults", defaults);
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
                          : power  ? "power-cycle reset queued"
                          : hard   ? "hardware reset queued"
                          : cold   ? "cold start queued"
                                   : "hot start queued");
    resp.AddItem("note", cold ? "the almanac is discarded; expect a slow reacquisition"
                              : "the almanac is kept; send {\"cold\":true} to discard it");
    return sendJson(req, resp, 200);
}

esp_err_t NtpWebServer::gps_survey_post_handler(httpd_req_t *req)
{
    /* Survey-in, then hold the position fixed. The survey ends only when both the
     * duration and the accuracy limit are met, and the limit is compared with the
     * receiver's mean variance, so a limit below the site's scatter never ends. */
    JsonWrapper doc;
    if (!readJsonBodyTimed(req, doc)) {
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

    /* Hours, 2 to 48. */
    double hours = 24;
    doc.GetField("hours", hours);
    if (hours < 2 || hours > 48) {
        return sendJsonError(req, 400, "hours must be 2..48");
    }
    /* Default 30 m: this site's scatter measured about 26 m, and a 3 m limit
     * never completed. */
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

    /* PENDING_VERIFY until eth_link.cpp confirms it; an unconfirmed image is
     * rolled back at the next reset. */
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    doc.AddItem("confirmed", running != nullptr &&
                                 esp_ota_get_state_partition(running, &state) == ESP_OK &&
                                 state == ESP_OTA_IMG_VALID);

    /* `ota` false: no second slot, so POST /firmware is refused. */
    doc.AddItem("ota", target != nullptr);
    if (target != nullptr) {
        doc.AddItem("next_partition", target->label);
        doc.AddItem("slot_size", static_cast<double>(target->size));
    } else {
        doc.AddNull("next_partition");
    }

    return sendJson(req, doc, 200);
}

/* Writes the body to the inactive slot and reboots into it. esp_ota_end()
 * checks it is an app image for this chip; a valid but bad image is caught by
 * the rollback in eth_link.cpp. No authentication: a trusted LAN is assumed. */
bool NtpWebServer::readJsonBodyTimed(httpd_req_t *req, JsonWrapper &out)
{
    /* Budgets for a caller that has stopped sending. They are checked only when
     * httpd_req_recv() times out (5 s), so "gone" is noticed at 10 s. */
    constexpr int64_t kBodyIdleUs  = 6000000;
    constexpr int64_t kBodyTotalUs = 10000000;
    constexpr size_t  kBodyMax     = 4096;

    if (req->content_len <= 0) {
        return sendJsonError(req, 400, "JSON body required") == ESP_OK;
    }
    if (static_cast<size_t>(req->content_len) > kBodyMax) {
        return sendJsonError(req, 413, "request body too large") == ESP_OK;
    }

    std::string   body(static_cast<size_t>(req->content_len), '\0');
    const int64_t started = esp_timer_get_time();
    int64_t       last    = started;
    size_t        got     = 0;
    while (got < body.size()) {
        const int r = httpd_req_recv(req, body.data() + got, body.size() - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            const int64_t nowUs = esp_timer_get_time();
            if (nowUs - last > kBodyIdleUs || nowUs - started > kBodyTotalUs) {
                /* Reply 408 and let the server close, rather than drain the body. */
                ESP_LOGW(TAG, "body stalled at %zu/%zu bytes, giving up", got, body.size());
                return sendJsonError(req, 408, "request body stalled") == ESP_OK;
            }
            continue;
        }
        if (r <= 0) {
            return sendJsonError(req, 400, "request body truncated") == ESP_OK;
        }
        got += static_cast<size_t>(r);
        last = esp_timer_get_time();
    }

    /* JsonWrapper cannot test for an object, so a throwaway parse does. */
    const cJSON *parsed   = cJSON_Parse(body.c_str());
    const bool   isObject = parsed != nullptr && cJSON_IsObject(parsed);
    cJSON_Delete(const_cast<cJSON *>(parsed));
    if (!isObject) {
        return sendJsonError(req, 400, "invalid JSON (expected an object)") == ESP_OK;
    }
    out = JsonWrapper::Parse(body);
    return true;
}

esp_err_t NtpWebServer::eth_restart_post_handler(httpd_req_t *req)
{
    /* Restart the MAC remotely, to exercise the stall recovery. Reply first. */
    JsonWrapper resp;
    resp.AddItem("restarts_so_far", static_cast<double>(EthernetLink::instance().restarts()));
    resp.AddItem("down_ms", 100.0);
    const esp_err_t err = sendJson(req, resp, 200);
    if (err != ESP_OK) {
        return err;
    }

    /* Long enough for the reply to leave. */
    vTaskDelay(pdMS_TO_TICKS(100));
    const esp_err_t restartErr = EthernetLink::instance().restart("http hook");
    ESP_LOGW(TAG, "POST /eth/restart from %s: %s", callerAddress(req).c_str(),
             esp_err_to_name(restartErr));
    return ESP_OK;
}

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
        /* Refused until this image has confirmed itself. */
        return sendJsonError(req, 409, "running image unconfirmed: retry once it has a lease");
    }
    if (err != ESP_OK) {
        return sendJsonError(req, 500, esp_err_to_name(err));
    }

    /* Abandon an upload idle this long; it blocks the only httpd task. */
    constexpr int64_t kOtaIdleUs = 15000000;

    char    buf[kOtaChunkBytes];
    int     remaining = req->content_len;
    int     written   = 0;
    int64_t lastData  = esp_timer_get_time();
    while (remaining > 0) {
        const int got = httpd_req_recv(req, buf, std::min(remaining, kOtaChunkBytes));
        if (got == HTTPD_SOCK_ERR_TIMEOUT) {
            if (esp_timer_get_time() - lastData > kOtaIdleUs) {
                esp_ota_abort(handle);
                ESP_LOGW(TAG, "firmware upload stalled at %d bytes, abandoned", written);
                return sendJsonError(req, 408, "request body stalled");
            }
            continue;
        }
        lastData = esp_timer_get_time();
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
        written += got;
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

    /* Let the reply leave before rebooting. */
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}
