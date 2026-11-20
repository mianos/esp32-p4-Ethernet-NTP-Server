#include "console.h"

#include <cstdio>
#include <cstring>
#include <string>

#include "esp_console.h"
#include "esp_log.h"

/* A C header with no extern "C" guard of its own. */
extern "C" {
#include "console_private.h"
}

#include "cmd_nvs.h"
#include "cmd_system.h"

#include "config_apply.h"
#include "eth_link.h"
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

/* The REPL runs under a fixed task name set by the console component, so it can
 * be found and suspended from inside one of its own commands. */
std::atomic<bool> s_watching{false};
/* Set while the console has silenced the log for its own output. */
bool            s_silenced  = false;
esp_log_level_t s_savedLevel = ESP_LOG_INFO;

/* Quiet console.
 *
 * The REPL task is suspended while this runs, so nothing prints a prompt and
 * linenoise stops redrawing -- but the port stays interactive: a line is read,
 * echoed, and handed to esp_console_run(), so every command still works. The
 * resume phrase is checked first because `console on` has to resume the REPL
 * rather than run inside this task.
 *
 * Nothing is persisted: a reboot always comes back with a live console. */
/* The prompt is a char buffer inside the console component's own struct, handed
 * to linenoise on every iteration, so rewriting it changes the next prompt.
 *
 * Suspending the REPL task to hide the prompt does not work: it is blocked in a
 * UART read whenever it is idle, and that outstanding read swallows whatever
 * arrives while it is suspended -- bytes vanish into a task that cannot run. The
 * prompt buffer can be rewritten on the live REPL instead, which leaves the
 * reading exactly where it was. */
esp_console_repl_com_t *s_replCom  = nullptr;
bool            s_quiet      = false;

/* repl_core is the first member of the struct, so the handle we were given is
 * its address. */
void setPrompt(const char *text)
{
    if (s_replCom != nullptr) {
        esp_console_setup_prompt(text, s_replCom);
    }
}

/* Two states, and the logging one is the default: the port is a log stream with
 * no prompt on it. `console on` is the debug state -- prompt, log silenced so
 * the session is readable. Commands work in both; only the prompt differs. */
int cmdConsole(void *, int argc, char **argv)
{
    const char *what = argc > 1 ? argv[1] : "";

    if (std::strcmp(what, "on") == 0) {
        if (!s_quiet) {
            std::printf("console is already up\n");
            return 0;
        }
        s_quiet = false;
        if (!s_silenced) {
            s_savedLevel = esp_log_level_get("*");
            s_silenced   = true;
        }
        esp_log_level_set("*", ESP_LOG_NONE);
        setPrompt("ntp>");
        std::printf("\nconsole up, log silenced ('console off' restores)\n");
        return 0;
    }

    if (std::strcmp(what, "off") == 0) {
        if (s_quiet) {
            std::printf("console is already down\n");
            return 0;
        }
        s_quiet = true;
        if (s_silenced) {
            esp_log_level_set("*", s_savedLevel);
            s_silenced = false;
        }
        setPrompt("");
        std::printf("console down, log back at its level ('console on' for the prompt)\n");
        return 0;
    }

    if (argc > 1) {
        std::printf("usage: console [on | off]\n");
        return 1;
    }
    std::printf("console %s%s\n", s_quiet ? "down (logging, no prompt)" : "up",
                s_silenced ? ", log silenced" : "");
    return 0;
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
    const esp_err_t started = esp_console_start_repl(handle);
    if (started != ESP_OK) {
        return started;
    }

    /* Boot into the logging state: no prompt on the port, commands still run.
     * The prompt is blanked rather than the task suspended, and a blank prompt
     * is what `console on` restores. */
    s_replCom = reinterpret_cast<esp_console_repl_com_t *>(handle);
    s_quiet   = true;
    setPrompt("");
    std::printf("logging -- 'console on' for the prompt\r\n");
    std::fflush(stdout);
    return ESP_OK;
}
