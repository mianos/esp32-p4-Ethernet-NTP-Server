/* Console output over the network.
 *
 * There is no radio on this board and, once the box is out of reach, no USB
 * cable either: the only way to watch it is the LAN it already sits on. This
 * duplicates ESP_LOG to a UDP port so a phone or laptop on the WiFi can follow
 * it with `nc -ukl 5514`.
 *
 * The hook is esp_log_set_vprintf, which runs in whatever task is logging and
 * must not block: a UDP send with the netif busy can stall for milliseconds.
 * So the hook only formats into a stack buffer and pushes it at a stream
 * buffer; a task drains that and does the sending. When the buffer is full the
 * line is dropped rather than the logger waiting -- a lost log line is cheaper
 * than a stalled GPS task.
 *
 * Off unless both a host and a port are configured. Nothing here is a control
 * path: the console is unchanged and receives the same bytes.
 */
#include "net/net_log.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "app/settings.h"

namespace {

constexpr const char *TAG        = "net_log";
constexpr size_t      kRingBytes = 4096;
constexpr size_t      kLineMax   = 256;
constexpr UBaseType_t kTaskPrio  = 2;

StreamBufferHandle_t m_ring      = nullptr;
int                  m_sock      = -1;
sockaddr_in          m_to        = {};
bool                 m_installed = false;
vprintf_like_t       m_console   = nullptr;

/* The console still gets everything: this forwards to the previous hook first,
 * so a broken sink never costs the local output. */
int hook(const char *fmt, va_list args)
{
    char    line[kLineMax];
    va_list copy;
    va_copy(copy, args);
    const int n = std::vsnprintf(line, sizeof(line), fmt, copy);
    va_end(copy);

    if (n > 0 && m_ring != nullptr) {
        const size_t len =
            static_cast<size_t>(n) < sizeof(line) ? static_cast<size_t>(n) : sizeof(line) - 1;
        xStreamBufferSend(m_ring, line, len, 0); /* never waits */
    }
    return m_console != nullptr ? m_console(fmt, args) : n;
}

void drainTask(void *)
{
    char buf[kLineMax];
    while (true) {
        const size_t got = xStreamBufferReceive(m_ring, buf, sizeof(buf), portMAX_DELAY);
        if (got > 0 && m_sock >= 0 && m_to.sin_port != 0) {
            sendto(m_sock, buf, got, 0, reinterpret_cast<sockaddr *>(&m_to), sizeof(m_to));
        }
    }
}

}  // namespace

esp_err_t net_log::start()
{
    const Settings    &settings = Settings::instance();
    const std::string &host     = settings.logHost();
    unsigned long      port     = 0;
    if (host.empty() || std::sscanf(settings.logPort().c_str(), "%lu", &port) != 1 || port == 0 ||
        port > 65535) {
        /* Not configured -- which is also how a running sink is switched off.
         * This returned with the socket and the old address still in place, so
         * clearing logHost left the log going to it until the next boot. */
        m_to = {};
        return ESP_OK;
    }

    /* A name would need the resolver here, so only dotted quads are accepted
     * and /config says as much. */
    m_to            = {};
    m_to.sin_family = AF_INET;
    m_to.sin_port   = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &m_to.sin_addr) != 1) {
        ESP_LOGW(TAG, "log host '%s' is not a dotted quad: sink off", host.c_str());
        return ESP_OK;
    }

    /* Re-targeting an existing sink is just the address: a second task and
     * socket would double every line. */
    if (m_sock >= 0) {
        ESP_LOGI(TAG, "console now also to udp %s:%lu", host.c_str(), port);
        return ESP_OK;
    }

    m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_sock < 0) {
        ESP_LOGW(TAG, "socket: errno %d", errno);
        return ESP_FAIL;
    }

    m_ring = xStreamBufferCreate(kRingBytes, 1);
    if (m_ring == nullptr) {
        close(m_sock);
        m_sock = -1;
        return ESP_FAIL;
    }

    if (xTaskCreate(&drainTask, "netlog", 3072, nullptr, kTaskPrio, nullptr) != pdPASS) {
        ESP_LOGW(TAG, "no task");
        /* Undone, or the next call sees a socket and reports a sink that has
         * nothing draining it. */
        vStreamBufferDelete(m_ring);
        m_ring = nullptr;
        close(m_sock);
        m_sock = -1;
        return ESP_FAIL;
    }

    if (!m_installed) {
        m_console   = esp_log_set_vprintf(&hook);
        m_installed = true;
    }
    ESP_LOGI(TAG, "console also to udp %s:%lu", host.c_str(), port);
    return ESP_OK;
}
