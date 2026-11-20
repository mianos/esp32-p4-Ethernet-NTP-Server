#include "console.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_app_desc.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_timer.h"


#include "cmd_nvs.h"
#include "cmd_system.h"

#include "clock_source.h"
#include "config_apply.h"
#include "eth_link.h"
#include "gps_time.h"
#include "net_addr.h"
#include "net_state.h"
#include "settings.h"

namespace {

constexpr const char *TAG = "console";

/* Filled by start(), never at static-init time: Settings opens an NVS handle,
 * and NVS is not initialised until app_main has run. */
struct Context {
    Settings     *settings = nullptr;
    EthernetLink *link     = nullptr;
};

Context s_ctx;

/* The port has two states: a log stream, which is the default, or a prompt for
 * a debug session. `console on` means "give me the prompt" and holds the log
 * while it is up; `console off` returns to the stream.
 *
 * The console component's own REPL task is created but never started. It is
 * blocked on its start notification, which is the point: a REPL that is running
 * sits in a blocking UART read, and that pending read takes input out of the
 * driver for a task that may not be able to run it. Reading here, in one task
 * that dispatches through esp_console_run(), keeps exactly one reader on the
 * port. It also avoids linenoise's terminal probes -- the `ESC[6n` cursor
 * queries that arrive as input and get chewed as commands.
 *
 * The cost is line editing: backspace works, arrows and history do not. */
constexpr const char *kPrompt = "ntp> ";

bool s_prompted = false;
/* The level to come back to when the prompt goes away again. */
esp_log_level_t s_logLevel = ESP_LOG_INFO;

int cmdConsole(void *, int argc, char **argv)
{
    const char *what = argc > 1 ? argv[1] : "";

    if (std::strcmp(what, "on") == 0) {
        if (s_prompted) {
            std::printf("console is already up\n");
            return 0;
        }
        const esp_log_level_t now = esp_log_level_get("*");
        if (now != ESP_LOG_NONE) {
            s_logLevel = now;
        }
        s_prompted = true;
        esp_log_level_set("*", ESP_LOG_NONE);
        std::printf("console up: prompt, log held. 'console off' returns to the log.\n");
        return 0;
    }

    if (std::strcmp(what, "off") == 0) {
        if (!s_prompted) {
            std::printf("console is already down\n");
            return 0;
        }
        s_prompted = false;
        esp_log_level_set("*", s_logLevel);
        std::printf("console down: log back at its level. 'console on' for the prompt.\r\n");
        return 0;
    }

    if (argc > 1) {
        std::printf("usage: console [on | off]\n");
        return 1;
    }
    std::printf("console %s\n", s_prompted ? "up (prompt, log held)" : "down (log stream)");
    return 0;
}

/* The one reader. A prompt is printed only when one has been asked for, and the
 * line is dispatched through the same entry point the REPL uses, so the command
 * layer is unchanged. */
void readerTask(void *)
{
    std::string line;

    for (;;) {
        if (s_prompted) {
            std::printf("%s", kPrompt);
            std::fflush(stdout);
        }
        line.clear();

        for (;;) {
            char c = 0;
            if (read(STDIN_FILENO, &c, 1) != 1) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            if (c == '\r' || c == '\n') {
                break;
            }
            if (c == 0x7f || c == 0x08) {
                if (!line.empty()) {
                    line.pop_back();
                    std::fputs("\b \b", stdout);
                    std::fflush(stdout);
                }
                continue;
            }
            if (line.size() < 512) {
                line.push_back(c);
                std::putchar(c);
                std::fflush(stdout);
            }
        }

        std::printf("\r\n");
        if (!line.empty()) {
            int ret = 0;
            esp_console_run(line.c_str(), &ret);
        }
    }
}

void printAddressing()
{
    const Settings *settings = s_ctx.settings;
    if (settings == nullptr) {
        std::printf("settings not ready\n");
        return;
    }

    std::printf("mode    : %s\n", settings->ipMode().c_str());
    std::printf("address : %s\n",
                settings->ipAddress().empty() ? "(unset)" : settings->ipAddress().c_str());
    std::printf("gateway : %s\n",
                settings->ipGateway().empty() ? "(unset)" : settings->ipGateway().c_str());
    std::printf("dns     : %s\n", settings->ipDns().empty() ? "(unset)" : settings->ipDns().c_str());
    std::printf("link    : %s\n", net_state::linkUp() ? "up" : "down");
}

/* The one write path: a document, applyJson(), save, and the shared hook
 * sequence.
 *
 * Success is judged on the key's value afterwards, not on the blob having
 * changed. Setting something to what it already was is a legitimate no-op --
 * `net static` re-issued with the stored address has to keep going and set the
 * mode -- while a value applyJson refused leaves a different one behind. */
bool applyOne(const char *key, const std::string &value)
{
    if (s_ctx.settings == nullptr) {
        std::printf("not started yet\n");
        return false;
    }

    const std::string host = s_ctx.settings->hostname();

    JsonWrapper doc;
    doc.AddItem(key, value);
    s_ctx.settings->applyJson(doc);

    /* The only field applyJson is allowed to shorten. */
    const std::string wanted = std::strcmp(key, "hostname") == 0
                                   ? value.substr(0, Settings::kMaxHostnameLen)
                                   : value;
    std::string   now;
    s_ctx.settings->toJson().GetField(key, now);
    if (now != wanted) {
        std::printf("'%s' = '%s' unknown or rejected -- nothing changed\n", key, value.c_str());
        return false;
    }

    s_ctx.settings->save();
    s_ctx.settings->log();
    config_apply::afterChange(host, std::strcmp(key, "logHost") == 0 ||
                                        std::strcmp(key, "logPort") == 0);
    return true;
}

int cmdNet(void *, int argc, char **argv)
{
    if (argc == 1) {
        printAddressing();
        return 0;
    }

    if (std::strcmp(argv[1], "dhcp") == 0) {
        if (!applyOne("ipMode", "dhcp")) {
            return 1;
        }
        std::printf("mode dhcp: the client is requesting a lease\n");
        printAddressing();
        return 0;
    }

    if (std::strcmp(argv[1], "static") != 0) {
        std::printf("usage: net [dhcp | static <cidr> <gateway> [dns]]\n");
        return 1;
    }
    if (argc < 4) {
        std::printf("usage: net static <a.b.c.d/len> <gateway> [dns]\n");
        return 1;
    }

    esp_ip4_addr_t addr{};
    uint8_t        prefix = 0;
    if (!net_addr::parseCidr(argv[2], addr, prefix)) {
        std::printf("'%s' is not a.b.c.d/len with len 1..30\n", argv[2]);
        return 1;
    }
    esp_ip4_addr_t gw{};
    if (!net_addr::parseQuad(argv[3], gw)) {
        std::printf("gateway '%s' is not a dotted quad\n", argv[3]);
        return 1;
    }
    if (argc > 4) {
        esp_ip4_addr_t dns{};
        if (!net_addr::parseQuad(argv[4], dns)) {
            std::printf("resolver '%s' is not a dotted quad\n", argv[4]);
            return 1;
        }
        if (!applyOne("ipDns", argv[4])) {
            return 1;
        }
    }

    /* Mode last: nothing tries to program a half-written address. */
    if (!applyOne("ipAddress", argv[2]) || !applyOne("ipGateway", argv[3]) ||
        !applyOne("ipMode", "static")) {
        return 1;
    }
    std::printf("static address set -- this is the one that can strand the box\n");
    printAddressing();
    return 0;
}

/* Print, set or reset the whole config. */
int cmdConfig(void *, int argc, char **argv)
{
    if (s_ctx.settings == nullptr) {
        std::printf("not started yet\n");
        return 1;
    }

    if (argc == 1 || (argc > 1 && std::strcmp(argv[1], "raw") == 0)) {
        const std::string compact = s_ctx.settings->toJson().ToString();
        if (argc > 1) {
            std::printf("%s\n", compact.c_str());
            return 0;
        }
        /* Readable, because this is the one place a person reads the whole
         * config. cJSON is already a dependency of the settings store. */
        cJSON *root   = cJSON_Parse(compact.c_str());
        char  *pretty = root != nullptr ? cJSON_Print(root) : nullptr;
        if (pretty != nullptr) {
            std::printf("%s\n", pretty);
            cJSON_free(pretty);
        } else {
            std::printf("%s\n", compact.c_str());
        }
        cJSON_Delete(root);
        return 0;
    }

    if (std::strcmp(argv[1], "reset") == 0) {
        const std::string before = s_ctx.settings->hostname();
        const std::string boots  = s_ctx.settings->boots();
        s_ctx.settings->resetToDefaults();
        s_ctx.settings->setBoots(boots);
        s_ctx.settings->save();
        s_ctx.settings->log();
        config_apply::afterChange(before, true);
        std::printf("back to the compiled-in defaults\n");
        return 0;
    }

    if (std::strcmp(argv[1], "set") != 0) {
        std::printf("usage: config [raw | reset | set <key> <value>]\n");
        return 1;
    }
    if (argc < 4) {
        std::printf("usage: config set <key> <value>\n");
        return 1;
    }
    if (!applyOne(argv[2], argv[3])) {
        return 1;
    }
    std::printf("%s = %s\n", argv[2], argv[3]);
    return 0;
}

int cmdHostname(void *, int argc, char **argv)
{
    if (s_ctx.settings == nullptr) {
        std::printf("not started yet\n");
        return 1;
    }
    if (argc < 2) {
        std::printf("hostname: %s\n", s_ctx.settings->hostname().c_str());
        return 0;
    }
    /* applyJson truncates past the cap, so this only says what happened. */
    if (std::strlen(argv[1]) > Settings::kMaxHostnameLen) {
        std::printf("at most %u characters -- truncating\n",
                    static_cast<unsigned>(Settings::kMaxHostnameLen));
    }
    return applyOne("hostname", argv[1]) ? 0 : 1;
}

int cmdServe(void *, int argc, char **argv)
{
    if (s_ctx.settings == nullptr) {
        std::printf("not started yet\n");
        return 1;
    }
    if (argc < 2) {
        std::printf("serveNtp: %s\n", s_ctx.settings->serveNtp() ? "on" : "off");
        return 0;
    }
    const std::string value = argv[1];
    if (value != "on" && value != "off") {
        std::printf("usage: serve <on|off>\n");
        return 1;
    }
    return applyOne("serveNtp", value) ? 0 : 1;
}

int cmdTimesource(void *, int argc, char **argv)
{
    if (s_ctx.settings == nullptr) {
        std::printf("not started yet\n");
        return 1;
    }
    if (argc < 2) {
        std::printf("timeSource: %s\n", s_ctx.settings->timeSourceText().c_str());
        return 0;
    }
    const std::string value = argv[1];
    if (value != "off" && value != "dhcp" && value != "static" && value != "gps" && value != "auto") {
        std::printf("usage: timesource <off|dhcp|static|gps|auto>\n");
        return 1;
    }
    return applyOne("timeSource", value) ? 0 : 1;
}

int cmdNtpServer(void *, int argc, char **argv)
{
    if (s_ctx.settings == nullptr) {
        std::printf("not started yet\n");
        return 1;
    }
    if (argc < 2) {
        std::printf("ntpServer: %s\n", s_ctx.settings->ntpServer().c_str());
        return 0;
    }
    return applyOne("ntpServer", argv[1]) ? 0 : 1;
}

int cmdLog(void *, int argc, char **argv)
{
    if (s_ctx.settings == nullptr) {
        std::printf("not started yet\n");
        return 1;
    }
    if (argc < 3) {
        std::printf("log sink: %s:%s\n", s_ctx.settings->logHost().c_str(),
                    s_ctx.settings->logPort().c_str());
        return 0;
    }
    return applyOne("logHost", argv[1]) && applyOne("logPort", argv[2]) ? 0 : 1;
}

/* What the box is doing, in one screen. The console's value is that it works
 * when the network does not, so the first thing it should be able to do is say
 * what is wrong -- without this you read the log and guess. Read-only: every
 * line here is something the HTTP surface also reports. */
int cmdStatus(void *, int, char **)
{
    if (s_ctx.settings == nullptr) {
        std::printf("not started yet\n");
        return 1;
    }
    const esp_app_desc_t *desc = esp_app_get_description();
    std::printf("version  : %s\n", desc != nullptr ? desc->version : "?");
    std::printf("uptime   : %lld s, %s boots\n",
                static_cast<long long>(esp_timer_get_time() / 1000000),
                s_ctx.settings->boots().c_str());
    std::printf("link     : %s\n", net_state::linkUp() ? "up" : "down");
    printAddressing();
    const std::string global = net_state::ipv6Global();
    std::printf("ipv6     : %s / %s\n", net_state::ipv6LinkLocal().c_str(),
                global.empty() ? "(no global)" : global.c_str());

    const GpsTime::Status     gps = GpsTime::instance().status();
    const GpsTime::ReplyFacts rf  = GpsTime::instance().replyFacts();
    std::printf("fix      : %u, %u satellites, anchored %s\n", static_cast<unsigned>(gps.fixType),
                static_cast<unsigned>(gps.satellites), gps.anchored ? "yes" : "no");
    std::printf("mode     : %s%s\n", gps.timeMode.c_str(),
                gps.holdValidated ? " (survey-validated)" : "");
    std::printf("serving  : owner %s for %u s, stratum %u%s, locked %s\n",
                ClockSource::name(ClockSource::instance().owner()),
                static_cast<unsigned>(ClockSource::instance().ageS()),
                static_cast<unsigned>(rf.stratum), rf.demoted ? " (demoted)" : "",
                gps.locked ? "yes" : "no");
    std::printf("dispersion: %.1f us\n", static_cast<double>(rf.dispersionNs) / 1000.0);
    return 0;
}

/* The link, and the recovery for it. A wedged MAC is the one fault that takes
 * the HTTP surface with it, so it needs a way back that does not use it. */
int cmdEth(void *, int argc, char **argv)
{
    if (s_ctx.link == nullptr) {
        std::printf("not started yet\n");
        return 1;
    }
    if (argc == 1) {
        std::printf("link:%s  restarts:%u\n", net_state::linkUp() ? "up" : "down",
                    static_cast<unsigned>(s_ctx.link->restarts()));
        printAddressing();
        return 0;
    }
    if (std::strcmp(argv[1], "restart") != 0) {
        std::printf("usage: eth [restart]\n");
        return 1;
    }
    const esp_err_t err = s_ctx.link->restart("console");
    std::printf("restart: %s (restarts now %u)\n", esp_err_to_name(err),
                static_cast<unsigned>(s_ctx.link->restarts()));
    return err == ESP_OK ? 0 : 1;
}

/* The receiver's own controls. The settings behind them are reachable through
 * `config set`; what is not is *starting* something -- and a survey that will
 * never converge is exactly what wants stopping from a cable. */
int cmdGps(void *, int argc, char **argv)
{
    GpsTime &gps = GpsTime::instance();
    if (argc == 1) {
        const GpsTime::Status st = gps.status();
        std::printf("fix %u, %u satellites, anchored %s, mode %s\n",
                    static_cast<unsigned>(st.fixType), static_cast<unsigned>(st.satellites),
                    st.anchored ? "yes" : "no", st.timeMode.c_str());
        std::printf("survey: %s, %.0f s of %.0f s, %u obs\n",
                    st.svinValid ? "valid" : "not validated", st.svinAgeS, st.svinTargetS,
                    static_cast<unsigned>(st.svinObs));
        return 0;
    }
    if (std::strcmp(argv[1], "survey") == 0) {
        const char *what = argc > 2 ? argv[2] : "";
        if (std::strcmp(what, "stop") == 0) {
            gps.requestSurvey(false);
            std::printf("survey stop queued: back to a moving position\n");
            return 0;
        }
        if (std::strcmp(what, "fix") == 0) {
            gps.requestFixPosition();
            std::printf("lock queued: the receiver's own mean becomes the held position\n");
            return 0;
        }
        std::printf("usage: gps survey <stop|fix>  (parameters: config set gpsSurvey H,MM)\n");
        return 1;
    }
    if (std::strcmp(argv[1], "reset") == 0) {
        const char *what = argc > 2 ? argv[2] : "soft";
        if (std::strcmp(what, "soft") == 0) {
            gps.requestReset();
        } else if (std::strcmp(what, "hard") == 0) {
            gps.requestReset(false, true);
        } else if (std::strcmp(what, "power") == 0) {
            gps.requestReset(false, false, true);
        } else if (std::strcmp(what, "cold") == 0) {
            gps.requestReset(true);
        } else {
            std::printf("usage: gps reset [soft|hard|power|cold]\n");
            return 1;
        }
        std::printf("reset (%s) queued\n", what);
        return 0;
    }
    if (std::strcmp(argv[1], "ant") == 0) {
        const char *what = argc > 2 ? argv[2] : "";
        if (std::strcmp(what, "on") != 0 && std::strcmp(what, "off") != 0) {
            std::printf("usage: gps ant <on|off>\n");
            return 1;
        }
        gps.requestAntennaBias(std::strcmp(what, "on") == 0);
        std::printf("antenna bias %s queued\n", what);
        return 0;
    }
    if (std::strcmp(argv[1], "gnss") == 0) {
        if (argc < 3) {
            std::printf("usage: gps gnss <hex mask>\n");
            return 1;
        }
        char         *end = nullptr;
        const long    mask = std::strtol(argv[2], &end, 16);
        if (end == argv[2] || mask <= 0 || mask > 0xFF) {
            std::printf("'%s' is not a hex mask, 1..ff\n", argv[2]);
            return 1;
        }
        gps.requestGnss(static_cast<uint8_t>(mask));
        std::printf("constellation mask 0x%02lx queued\n", mask);
        return 0;
    }
    if (std::strcmp(argv[1], "suppress") == 0) {
        if (argc < 3) {
            std::printf("usage: gps suppress <seconds>  (0 to stop)\n");
            return 1;
        }
        const uint32_t seconds = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
        gps.suppressPps(seconds);
        std::printf("edges %s\n", seconds == 0 ? "counted again" : "ignored: this is a holdover "
                                                                    "and the reply says so");
        return 0;
    }
    if (std::strcmp(argv[1], "defaults") == 0) {
        gps.requestDefaults();
        std::printf("receiver configuration clear queued\n");
        return 0;
    }
    std::printf("usage: gps [survey <stop|fix> | reset [soft|hard|power|cold] | ant <on|off> | "
                "defaults]\n");
    return 1;
}

void add(const char *name, const char *help, const char *hint,
         esp_console_cmd_func_with_context_t func)
{
    const esp_console_cmd_t cmd = {
        .command        = name,
        .help           = help,
        .hint           = hint,
        .func           = nullptr,
        .argtable       = nullptr,
        .func_w_context = func,
        .context        = &s_ctx,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

}  // namespace

esp_err_t console::start()
{
    s_ctx.settings = &Settings::instance();
    s_ctx.link     = &EthernetLink::instance();

    esp_console_repl_config_t repl = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl.prompt             = "ntp>";
    repl.max_cmdline_length = 512;
    /* The default 4096 is thin for stdio plus argtable. */
    repl.task_stack_size    = 6144;

    esp_console_register_help_command();
    register_system_common();
    register_nvs();

    add("net", "Show or set addressing", "[dhcp | static <cidr> <gateway> [dns]]", cmdNet);
    add("config", "Show (pretty, or raw), set or reset the stored config",
        "[raw | reset | set <key> <value>]", cmdConfig);
    add("console", "Debug console on (prompt, log silenced) or off (log stream)",
        "[on | off]", cmdConsole);
    add("hostname", "Show or set the DHCP hostname", "[name]", cmdHostname);
    add("serve", "Serve NTP on udp/123", "<on|off>", cmdServe);
    add("timesource", "Where the clock comes from", "<off|dhcp|static|gps|auto>", cmdTimesource);
    add("ntpserver", "Upstream server for timeSource=static", "[host]", cmdNtpServer);
    add("log", "UDP log sink", "[<ip> <port>]", cmdLog);
    add("status", "One screen: link, addressing, fix, serving", "", cmdStatus);
    add("eth", "Link state, or restart the MAC", "[restart]", cmdEth);
    add("gps", "Receiver state, survey, reset, antenna, mask, holdover test",
        "[survey <stop|fix> | reset [soft|hard|power|cold] | ant <on|off> | gnss <hex> | "
        "suppress <s> | defaults]", cmdGps);

    /* Whichever transport the console is configured for. It is UART0 on this
     * board -- the USB-C port is a CH343 bridge wired to it, the same cable
     * esptool flashes through -- but the chain costs nothing and keeps the file
     * honest if the wiring ever changes. */
    esp_console_repl_t *handle = nullptr;
    esp_err_t           err    = ESP_FAIL;
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    const esp_console_dev_uart_config_t hw = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    err = esp_console_new_repl_uart(&hw, &repl, &handle);
#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    const esp_console_dev_usb_serial_jtag_config_t hw =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    err = esp_console_new_repl_usb_serial_jtag(&hw, &repl, &handle);
#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    const esp_console_dev_usb_cdc_config_t hw = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    err = esp_console_new_repl_usb_cdc(&hw, &repl, &handle);
#endif
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "repl: %s", esp_err_to_name(err));
        return err;
    }
    /* Deliberately not esp_console_start_repl(): the REPL task stays parked on
     * its notification while the reader below owns the port. */
    (void)handle;

    if (xTaskCreate(&readerTask, "console_in", 6144, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "no reader task");
        return ESP_ERR_NO_MEM;
    }
    std::printf("logging -- 'console on' for the prompt\r\n");
    std::fflush(stdout);
    return ESP_OK;
}
