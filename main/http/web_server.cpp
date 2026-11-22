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

    /* A table, kept in columns by hand: the formatter packs it. */
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
    /* Why this boot happened, and how many there have been. A power cut used to
     * be inferable only from what had silently gone missing. */
    json.AddItem("reset_reason", resetReasonText());
    json.AddItem("boots", std::strtoul(Settings::instance().boots().c_str(), nullptr, 10));
    json.AddItem("heap_free", static_cast<double>(esp_get_free_heap_size()));

    const Settings &settings = Settings::instance();
    const NtpTime  &ntp      = NtpTime::instance();

    json.AddItem("hostname", settings.hostname());
    json.AddItem("link_up", net_state::linkUp());
    /* Which addressing mode is configured, so a box on a static address says so
     * rather than being inferred from a lease that will never arrive. */
    json.AddItem("ip_mode", Settings::instance().ipMode());

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
     * With the receiver serving there is no upstream to name: its own state
     * replaces that half, and ntp_synced means "the source that is serving is
     * locked", which for the GPS is a fix plus a fresh PPS edge.
     *
     * ntp_source is the *configured* value and so reads "auto" on a box that
     * may be on either; time_owner says which one actually has the clock, and
     * ntp_client what the client is doing. Reporting only the setting would
     * leave a fallback invisible in exactly the field a reader looks at. */
    const ClockSource &arbiter = ClockSource::instance();
    json.AddItem("ntp_source", Settings::timeSourceName(settings.timeSource()));
    json.AddItem("ntp_client", ntp.sourceName());
    json.AddItem("time_owner", ClockSource::name(arbiter.owner()));
    json.AddItem("gps_fallback", arbiter.owner() != ClockOwner::Gps);
    /* Cumulative, so it stays after the fallback ends; the age is per episode. */
    json.AddItem("fallback_count", static_cast<double>(arbiter.engagements()));
    if (arbiter.owner() != ClockOwner::Gps) {
        json.AddItem("fallback_age_s", static_cast<double>(arbiter.ageS()));
    }

    /* The receiver's headline state whatever is serving. On a fallback these
     * three are the answer to "is it the antenna, or the sky?", and they used
     * to vanish exactly then because they sat in the receiver's own branch. */
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
        /* Time validity, edge count and whether an edge has ever been paired
         * with a date. A 3D fix with invalid time is the state that quietly
         * serves rubbish, and `locked` alone hides it by folding all three
         * together; with ppsCount the difference between "the edges stopped"
         * and "the edges never started" also stops reading the same. */
        json.AddItem("gps_time_valid", gps.timeValid);
        json.AddItem("gps_pps_count", static_cast<double>(gps.ppsCount));
        json.AddItem("gps_anchored", gps.anchored);
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
            json.AddItem("gps_pps_suppressed_s", static_cast<double>(gps.ppsSuppressUs / 1000000));
        }
        /* What the last holdover actually cost, measured when the edge came
         * back: how long it was, and how far the servo's prediction was out.
         * Absent until there has been one, and both are named for it -- the
         * error used to be `gps_holdover_error_us`, which read as a live figure
         * and was alarming to find sitting on a box that was anchored. */
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
        /* The freshness window, not "was set once at boot": under auto this
         * branch is also reached while a fallback is being served, and a stale
         * sync has to read as unsynced or a reader would trust it. */
        json.AddItem("ntp_synced", arbiter.upstreamLocked());
        if (ntp.lastSync() != 0) {
            json.AddItem("last_sync", static_cast<double>(ntp.lastSync()));
        }
    }

    /* The responder's half of the story: whether it is listening, what it would
     * claim if asked right now, and how much it has answered since boot. */
    const NtpServer &server = NtpServer::instance();
    json.AddItem("serve_ntp", server.running());
    /* The hardware timestamp path. Counters rather than a claim: a counter that
     * is not moving is the answer, and a zero here means the responder is back
     * on software stamps. */
    const ptp::Stats stamps = ptp::stats();
    json.AddItem("phc", ptp::hardware());
    json.AddItem("hw_rx_stamps", static_cast<double>(stamps.rxStamps));
    json.AddItem("hw_tx_stamps", static_cast<double>(stamps.txStamps));
    json.AddItem("hw_tx_missing", static_cast<double>(stamps.txMissing));
    json.AddItem("hw_tx_errors", static_cast<double>(stamps.txErrors));
    /* PTP. The mode is the setting; the counters say whether anything is
     * actually going out, which is the part a setting cannot. */
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
        /* What the leap field of a reply would say right now: 1 or 2 for the
         * minute before a leap, 3 when the reference cannot vouch for the clock
         * at all, and 0 the rest of the time. */
        json.AddItem("ntp_leap", static_cast<double>(ref.locked ? ref.leap : 3));
        /* True when the reference is holding but its own budget says it is not
         * fit to serve: stratum 16 and leap 3 are then deliberate, not a lost
         * fix. */
        json.AddItem("ntp_demoted", ref.demoted);
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
        /* The socket transmit-lag estimator, and how many replies actually had
         * it applied: the raw send path is fast enough not to, so on a healthy
         * box the correction describes the sendto() fallback and the count says
         * nothing is using it. */
        json.AddItem("ntp_tx_correction_us", server.txLagUs());
        json.AddItem("ntp_tx_corrected", static_cast<double>(server.txCorrected()));
        json.AddItem("ntp_tx_scatter_us", server.txScatterUs());
        /* What dispersion rests on: the measured drift of the served time on
         * answers that went out, which is the number a client's bound is made
         * of. Auditable, so the claim can be checked against the log. */
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
    /* Shared helper: rejects an empty, oversized (>4 KB) or non-object body
     * with the JSON error shape already sent, and parses it otherwise. */
    JsonWrapper doc;
    if (!readJsonBodyTimed(req, doc)) {
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
    const bool logChanged = doc.ContainsField("logHost") || doc.ContainsField("logPort");
    if (settings.toJson().ToString() != before) {
        settings.save();
    }
    settings.log();

    /* Shared with the console: a changed address, hostname or timeSource has to
     * take effect now rather than at the next boot, and both surfaces have to
     * do it in the same order. */
    config_apply::afterChange(hostname_before, logChanged);
    ESP_LOGI(TAG, "POST /config from %s", caller.c_str());

    return sendJson(req, settings.toJson(), 200);
}

esp_err_t NtpWebServer::config_reset_post_handler(httpd_req_t *req)
{
    const std::string caller = callerAddress(req);

    Settings         &settings        = Settings::instance();
    const std::string hostname_before = settings.hostname();
    /* Defaults do not include the boot count: it describes the device, not the
     * configuration, and a reset that zeroes it hides the reboot it precedes. */
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

/* POST /ntp/txlag {"enabled":true|false}: turn the transmit correction off to
 * measure what it is worth. Disabling it drops the receive lag back to its floor
 * too, because both come from one measurement. */
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

    /* The GPS is not polled: it reports once a second whether anyone asks or
     * not, so "sync now" has nothing to trigger. It still has an answer worth
     * giving -- is the clock disciplined at this moment -- so the same reply
     * shape is built from the receiver's state rather than refused.
     *
     * While the receiver is the one serving, not the setting: under auto it
     * keeps the clock until the fallback's upstream has synced, and a request
     * in that window is answered here rather than polling a client that may not
     * be running yet. off/dhcp/static are never the receiver's, so they keep the
     * 409 the client gives below. */
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
    /* The holdover test hook, and the reason it exists: the PPS line on this
     * board is soldered, so "unplug it and see what happens" is not available,
     * and a holdover nobody has exercised is a promise rather than a
     * measurement. This makes the interrupt ignore edges for a while; the
     * anchor stops advancing, dispersion grows, and the reference demotes
     * itself at the end of the window exactly as it would with a loose wire.
     * Nothing else changes, and /healthz reports the time left, so a reader can
     * tell a test from a fault. */
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
    /* The drain endpoint: GET /gps/samples?since=<seq>&max=<n>.
     *
     * Deliberately not inside /gps/info. That reply is rebuilt every poll and
     * is the bulk of the traffic; this one is bounded, cursor-addressed and
     * only asked for by the logger. since=0 means "from the oldest you have",
     * which is also what a client that has just restarted sends.
     *
     * lost is the ring telling the client how many epochs it dropped before
     * reaching the cursor. Non-zero is a real device-side hole and the client
     * writes one gap row for it; zero with a long absence is our own downtime,
     * which the ring has simply absorbed. */
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

    /* Built by hand rather than through JsonWrapper: an array of arrays with
     * nine fixed fields is a fraction of the bytes of 120 named objects, and
     * the wrapper would add each item one allocation at a time. The ninth is
     * the bound the reference was stating for that second, so the archive can
     * be read against the offset rather than only beside it. */
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

/* Every diagnostic the receiver has, in one place. Both endpoints below serve
 * this same document: /gps/info strips it to what a poller reads, /gps/bench
 * sends all of it. One builder means a new field is added once, and which
 * endpoint carries it is one line in kPolled rather than a second copy of the
 * emitter that drifts.
 *
 * The split exists because this grew from a bench endpoint into the polled one
 * as well, and 50-odd keys that only a human reads were going out every 2 s. */
static void emitGpsInfo(const GpsTime::Status &gps, JsonWrapper &doc)
{
    /* Who owns the clock. The receiver's own statistics read the same during a
     * fallback as before it, so without this a chart cannot tell a working
     * receiver from one whose offsets are being measured against somebody
     * else's clock. */
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

    /* The servo and what it buys. */
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
        /* Usually absent. Present only while the pulse position is unobserved,
         * so a large dispersion can be told from a bad clock. */
        if (gps.pulseUnobservedNs > 0) {
            doc.AddItem("pulse_unobserved_us", static_cast<double>(gps.pulseUnobservedNs) / 1000.0);
        }
    }
    /* The pulse's own quality. The jitter is OURS -- the spread of our own
     * stamping from the rate fit -- and was gated behind timTpCount with the two
     * TIM-TP figures, which hid it forever on a module that refuses the message
     * (this one does). qErr and onSecond genuinely need TIM-TP; the jitter does
     * not, so it stands alone. */
    doc.AddItem("tp_jitter_ns", static_cast<double>(gps.jitterNs));
    /* How long after the edge the frame that dates it arrived. This is the
     * margin the pairing window is spending: a max far below it means the
     * window can be tightened, and a max approaching it means the window is
     * already the limit and a late sentence is the fault to chase. The max is
     * the number to read -- a mean hides the tail that decides whether a
     * sentence could pair with the wrong edge. */
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
    /* Dynamic model and UTC standard: both defaults are wrong for a mast, so
     * whether the assertion took is part of what this endpoint answers. */
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
    doc.AddItem("edge_outliers", static_cast<double>(gps.edgeOutliers));
    doc.AddItem("status_seq", static_cast<double>(gps.snapshotSeq));
    doc.AddItem("status_age_ms", static_cast<double>(gps.snapshotAgeUs) / 1000.0);
    /* Edges the trust test refused outright, and whether the newest one passed.
     * A spike is an edge that landed a millisecond or more from where it was
     * expected, and an outlier one refused for being 20 us from the fit; a reject
     * is one that was never used, so the two are the difference between a pulse
     * with noise on it and a pulse that is not keeping time. */
    doc.AddItem("edge_rejects", static_cast<double>(gps.edgeRejects));
    doc.AddItem("last_edge_trusted", gps.lastEdgeTrusted);
    doc.AddItem("tie_max_ns", gps.tieMaxNs);
    doc.AddItem("tie_rms_ns", gps.tieRmsNs);
    /* How long the edge waited for its interrupt, captured in hardware at the
     * edge. The same question as tie_* asks, but of the stamp rather than the
     * fit; min is the fixed path, and the spread above it is what matters. */
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
    /* Where the pulse sits and where the solution epoch sits, which are not the
     * same question and, in fixed mode, are not both zero: `pvt_nano_ns` is
     * NAV-PVT's own offset into the second it names and `pulse_offset_ns` is
     * TIM-TP's statement about the pulse. A standing offset here with the offsets
     * elsewhere near zero means the two agree with each other and disagree with
     * UTC. */
    doc.AddItem("pvt_nano_ns", gps.pvtNanoNs);
    doc.AddItem("pulse_offset_ns", gps.pulseOffsetNs);
    doc.AddItem("pulse_on_second", gps.pulseOnSecond);
    doc.AddItem("hold_wanted", gps.holdWanted);
    doc.AddItem("hold_assert_in_s", gps.holdAssertInS);
    doc.AddItem("hold_ready", gps.holdReady);
    doc.AddItem("last_edge_age_ms", gps.lastEdgeAgeMs);

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
        doc.AddItem("user_delay_ns", gps.userDelayNs);
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
        /* "last_", not "holdover_": this is a record of the previous outage, not
         * the age of the current edge. /gps/pps publishes that one, and the two
         * were read as the same number -- an 80 s scar in this field looks
         * exactly like a clock that is 80 s into a holdover right now. */
        doc.AddItem("last_holdover_s", static_cast<double>(gps.lastHoldoverUs) / 1e6);
        doc.AddItem("last_holdover_error_us", static_cast<double>(gps.lastHoldoverErrNs) / 1000.0);
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
}

/* What /gps/info keeps: every key something reads, plus the five the standing
 * watch on this box is kept in. Everything else is /gps/bench. */
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
    /* Kept although nothing parses them yet: the trust test and the pairing
     * margin are what the operator watches after a pulse fault. */
    "edge_rejects",
    "last_edge_trusted",
    "pair_age_us",
    "pair_age_max_us",
    "pair_age_s",
    /* Whether the receiver still owns the clock, which every offset in the
     * document is measured against. */
    "time_owner",
    "gps_fallback",
};

/* Drop every key the polled endpoint has no use for. The names are collected
 * before anything is deleted: cJSON's item pointers are spliced by the delete. */
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

    /* Print, strip, print again would cost a parse; the wrapper hands the tree
     * over instead and the strip works on it in place. */
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
    /* Everything, including the configuration echoes: what the module says it
     * is, which messages it took, what the pins are doing. For a bench session
     * or a bug report, not for a 2 s poll. */
    const GpsTime::Status gps = GpsTime::instance().status();
    JsonWrapper           doc;
    emitGpsInfo(gps, doc);
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
    if (!readJsonBodyTimed(req, doc)) {
        return ESP_FAIL;
    }

    /* Two independent jobs on one endpoint: the bias switch, and the lead's
     * delay. Either may be present. */
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
    /* 200 us is more than 40 km of cable; below that the resolution of the
     * receiver's own solution is the limit, not the number. */
    if (haveDelay && (delayNs < 0 || delayNs > 200000)) {
        return sendJsonError(req, 400, "delay_ns must be 0..200000");
    }
    /* Short enough to be an edge, long enough for a scope. */
    if (havePulse && (pulseMs < 1 || pulseMs > 500)) {
        return sendJsonError(req, 400, "pulse_ms must be 1..500");
    }
    /* 10 ms of calibration is more than any receiver should need. */
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
        /* Coax is about 5 ns per metre whatever its loss class (velocity factor
         * 0.66, and the spread across real cables is under 20%). The delay is
         * taken off the pulse, which is the same correction whether the extra
         * metres are in the lead or the antenna is simply further away. */
        /* setX() only changes the object; save() is what reaches NVS, and
         * without it the value is back to zero at the next boot and the pulse
         * reverts with it. */
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
    if (!readJsonBodyTimed(req, doc)) {
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
    /* The host side alone, deliberately out of step with the receiver. The scan
     * puts the two back together within a few seconds, which is both the test
     * for that path and the way out of a line that has been desynchronised by
     * something that was not this firmware. */
    JsonWrapper doc;
    if (!readJsonBodyTimed(req, doc)) {
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
    bool cold     = false;
    bool hard     = false;
    bool power    = false;
    bool defaults = false;
    /* A body that is there has to be readable. Falling through on one that was
     * too long, cut short or not JSON queued a hot start and answered 200 to a
     * caller who had asked for something else. */
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
bool NtpWebServer::readJsonBodyTimed(httpd_req_t *req, JsonWrapper &out)
{
    /* Both budgets are about a caller that has stopped sending, not about a slow
     * one. They are only evaluated when httpd_req_recv() returns a timeout, so
     * the socket's own recv_wait_timeout (5 s here) is their granularity: an
     * idle budget below that would never be reached before the total one, and
     * the observed abandonment is the first multiple of the socket timeout past
     * either. Six seconds idle and ten total therefore mean "gone" is noticed at
     * ten, and a 4 kB body over a congested segment still fits. */
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
                /* The rest of the body is still in the socket; say 408 and let
                 * the server close rather than draining bytes nobody wants. */
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

    /* Objectness is tested on a throwaway parse because JsonWrapper has no
     * predicate for it, and a bare array would otherwise reach handlers that
     * only ever GetField(). */
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
    /* The cable is usually out of reach of whoever is looking at this page, so
     * the wedge recovery needs a way to be proved without unplugging anything.
     *
     * The reply goes first and the MAC goes down a moment later: written the
     * other way round, the JSON would be racing the PHY into a stop. */
    JsonWrapper resp;
    resp.AddItem("restarts_so_far", static_cast<double>(EthernetLink::instance().restarts()));
    resp.AddItem("down_ms", 100.0);
    const esp_err_t err = sendJson(req, resp, 200);
    if (err != ESP_OK) {
        return err;
    }

    /* Long enough for the datagram above to leave the driver, short enough that
     * the caller is still holding the connection when the link returns. */
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
        /* This image has not confirmed itself yet, so it is not allowed to
         * write another one. Retrying in a few seconds is enough. */
        return sendJsonError(req, 409, "running image unconfirmed: retry once it has a lease");
    }
    if (err != ESP_OK) {
        return sendJsonError(req, 500, esp_err_to_name(err));
    }

    /* The same shape, and the only place in the image where the size of the
     * read is a design decision: esp_ota_write takes the chunk as it comes. */
    /* How long an upload may send nothing before it is abandoned. Handlers run
     * on the one httpd task, so a client that went away mid-body without a FIN
     * used to hold this loop -- and every other endpoint -- until a reboot. */
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

    /* Give the reply time to leave the socket before the reboot drops it. */
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}
