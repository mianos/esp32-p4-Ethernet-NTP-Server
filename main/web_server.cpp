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
        json.AddItem("gps_offset_us", static_cast<double>(gps.offsetUs));

        /* The servo, and what it buys: the rate it has measured for the
         * crystal, how many anchors that is based on, and the window that rate
         * is worth holding over for. Before the first fit the rate is 0 and
         * rate_n grows towards 60; after it, this number is the difference
         * between a 3-second holdover and a 10-minute one. */
        json.AddItem("gps_rate_ppm", static_cast<double>(gps.ratePpb) / 1000.0);
        json.AddItem("gps_rate_n", static_cast<double>(gps.rateSamples));
        if (gps.locked) {
            json.AddItem("gps_dispersion_us", static_cast<double>(gps.dispersionNs) / 1000.0);
            json.AddItem("gps_lock_window_s",
                         static_cast<double>(gps.lockWindowUs / 1000000));
        }
        /* What the receiver says about its own pulse, once it has said
         * anything: the quantisation it admits to, the spread our stamps
         * actually show, and whether the pulse is on the UTC second. These are
         * the numbers behind root delay and dispersion -- before TIM-TP they
         * were a constant and an assumption. */
        if (gps.timTpCount != 0) {
            if (gps.qErrValid) {
                json.AddItem("gps_tp_qerr_ns", static_cast<double>(gps.qErrNs));
            }
            json.AddItem("gps_tp_jitter_ns", static_cast<double>(gps.jitterNs));
            json.AddItem("gps_tp_on_second", gps.pulseOnSecond);
        }
        /* Which receiver this is, and whether it took the two configuration
         * messages that make the rest of these numbers mean anything. Read here
         * rather than from the boot log: the console drops bytes exactly when
         * both of these are decided. */
        if (!gps.moduleId.empty()) {
            json.AddItem("gps_module", gps.moduleId);
        }
        json.AddItem("gps_tp5_ok", !gps.tp5Rejected);
        /* Three states, because they mean different things: refused (no such
         * message on this firmware), accepted (it exists and still never
         * arrived -- something else is wrong), or neither (never asked). */
        json.AddItem("gps_timtp", gps.timTpRejected ? "rejected"
                                   : gps.timTpAcked  ? "accepted"
                                                     : "silent");

        /* The antenna group, for swapping leads and comparing them. `gps_ant`
         * is what the supervisor found (ok / short / open, or no reading), and
         * `gps_ant_power` what the bias tee is doing -- the two questions a
         * high-gain antenna with a preamp actually raises. `gps_cn0_*` is the
         * comparison metric: C/N0 on used satellites, not satellite count,
         * because a better antenna adds the marginal ones last. agc/noise/jam
         * are the overload tell-tales -- a preamp that is too strong for this
         * receiver drives agc down and the jamming state up while the C/N0
         * still looks flattering. */
        json.AddItem("gps_ant_cfg", gps.antCfgRejected ? "rejected"
                                    : gps.antCfgAcked  ? "accepted"
                                                       : "silent");
        if (gps.monHwValid) {
            static constexpr const char *kAntStatus[] = {"init", "unknown", "ok", "short", "open"};
            static constexpr const char *kAntPower[]  = {"off", "on", "unknown"};
            static constexpr const char *kJam[]       = {"unknown", "ok", "warning", "critical"};
            json.AddItem("gps_ant", gps.antStatus <= 4 ? kAntStatus[gps.antStatus] : "unknown");
            json.AddItem("gps_ant_power", gps.antPower <= 2 ? kAntPower[gps.antPower] : "unknown");
            json.AddItem("gps_noise_dbm", -static_cast<double>(gps.noisePerMs));
            json.AddItem("gps_agc", static_cast<double>(gps.agcCnt));
            json.AddItem("gps_jam_ind", static_cast<double>(gps.jamInd));
            json.AddItem("gps_jam", gps.jamState <= 3 ? kJam[gps.jamState] : "unknown");
            /* How stale the supervisor reading is, so a frozen set of these
             * cannot pass for a live one. */
            json.AddItem("gps_ant_age_s", static_cast<double>(gps.monHwAgeUs) / 1e6);
        }
        /* Absent until a NAV-SAT has been parsed, and stale-flagged after 5 s,
         * so a stopped message cannot pass an old C/N0 off as a live one. */
        if (gps.cn0Used != 0 || gps.navSatAgeUs >= 0) {
            json.AddItem("gps_cn0_max", static_cast<double>(gps.cn0Max));
            json.AddItem("gps_cn0_mean", static_cast<double>(gps.cn0Mean));
            json.AddItem("gps_cn0_used", static_cast<double>(gps.cn0Used));
            json.AddItem("gps_cn0_age_s", static_cast<double>(gps.navSatAgeUs) / 1e6);
        }
        /* The leap-second notice a reply would carry. leap_pending is what puts
         * the indicator at 1 (or 2); leap_in_s only exists while something is
         * actually pending. */
        if (gps.leapValid) {
            json.AddItem("gps_leap_s", static_cast<double>(gps.leapSeconds));
            const bool pending = gps.leapChange != 0 && gps.leapInS > 0;
            json.AddItem("gps_leap_pending", pending);
            if (pending) {
                json.AddItem("gps_leap_in_s", static_cast<double>(gps.leapInS));
            }
        }
        /* Only while the holdover test hook is armed, so an artificial state is
         * never mistaken for a fault. */
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
    resp.AddItem("note", "queued for the GPS task; gps_ant_power and gps_ant report the result");
    return sendJson(req, resp, 200);
}

esp_err_t NtpWebServer::gps_reset_post_handler(httpd_req_t *req)
{
    /* A controlled hot start. The receiver gets reconfigured on every boot and
     * has been worked hard on a bench; this restarts the navigation engine
     * without clearing the almanac, which is the cheap way back from a state it
     * will not leave on its own. */
    GpsTime::instance().requestReset();
    ESP_LOGW(TAG, "POST /gps/reset from %s: hot start", callerAddress(req).c_str());

    JsonWrapper resp;
    resp.AddItem("reset", "hot start queued");
    resp.AddItem("note", "the almanac is kept; a cold start needs u-center");
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


