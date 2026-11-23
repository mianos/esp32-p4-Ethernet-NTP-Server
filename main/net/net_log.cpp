/* Copies the log to a UDP host and port (follow it with `nc -ukl 5514`).
 *
 * The vprintf hook runs in the logging task and must not block, so it only
 * queues the line on a stream buffer, dropping it when full; a task sends.
 * Off unless both host and port are set. The console output is unchanged. */
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
#include "app/cores.h"
#include "app/text.h"

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

/* Forwards to the previous hook first, so the console always gets the line. */
__attribute__((format(printf, 1, 0))) int hook(const char *fmt, va_list args)
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
    /* The whole setting or nothing: sscanf read "5514x" as 5514. */
    const auto         port     = text::toNumber<unsigned>(settings.logPort());
    if (host.empty() || !port || *port == 0 || *port > 65535) {
        /* Not configured: this also switches a running sink off. */
        m_to = {};
        return ESP_OK;
    }

    /* Dotted quads only; no resolver here. */
    m_to            = {};
    m_to.sin_family = AF_INET;
    m_to.sin_port   = htons(static_cast<uint16_t>(*port));
    if (inet_pton(AF_INET, host.c_str(), &m_to.sin_addr) != 1) {
        ESP_LOGW(TAG, "log host '%s' is not a dotted quad: sink off", host.c_str());
        return ESP_OK;
    }

    /* A running sink only changes its address. */
    if (m_sock >= 0) {
        ESP_LOGI(TAG, "console now also to udp %s:%u", host.c_str(), *port);
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

    if (xTaskCreatePinnedToCore(&drainTask, "netlog", 3072, nullptr, kTaskPrio, nullptr,
                                cores::kWork) != pdPASS) {
        ESP_LOGW(TAG, "no task");

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
    ESP_LOGI(TAG, "console also to udp %s:%u", host.c_str(), *port);
    return ESP_OK;
}
