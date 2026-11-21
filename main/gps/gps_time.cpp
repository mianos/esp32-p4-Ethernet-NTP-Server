#include "gps/gps_internal.h"

#include "gps/pps_latency.h"

using namespace gps_detail;

GpsTime &GpsTime::instance()
{
    static GpsTime gps;
    return gps;
}

void GpsTime::requestAntennaBias(bool on)
{
    m_biasReq = on ? 1 : 0;
}

void GpsTime::requestGnss(uint8_t mask)
{
    m_gnssReqMask = mask;
    m_gnssReq     = true;
}

void GpsTime::requestSurvey(bool start, uint32_t durS, uint32_t accMm)
{
    m_svinDurS  = durS;
    m_svinAccMm = accMm;
    m_svinCmd   = start ? 1 : 2;
}

void GpsTime::requestFixPosition()
{
    m_svinCmd = 3;
}

void GpsTime::requestAntDelay()
{
    m_tp5Cmd = true;
}

/* Pulse width, as a fraction of the second. Kept in the object only: it is a
 * bench knob for the timepulse, not a site setting. */
void GpsTime::requestPulseMs(uint32_t ms)
{
    m_pulseMs = ms;
    m_tp5Cmd  = true;
}

/* Period in microseconds, so 1000000 is 1 Hz. Changing it is the structural
 * test of whether this frame reaches the hardware at all: the pulse then stops
 * matching the second and every anchor shifts by a known fraction. */
void GpsTime::requestPeriodUs(uint32_t us)
{
    m_periodUs = us;
    m_tp5Cmd   = true;
}

void GpsTime::requestBaud(uint32_t baud, bool moduleToo)
{
    m_baudReq       = baud;
    m_baudReqModule = moduleToo;
}

void GpsTime::requestDefaults()
{
    m_defaultsReq = true;
}

void GpsTime::requestReset(bool cold, bool hardware, bool powerCycle)
{
    m_resetCold  = cold;
    m_resetHard  = hardware;
    m_resetPower = powerCycle;
    m_resetReq   = true;
}

void IRAM_ATTR GpsTime::recordEdge(const int64_t edgeNs, const int64_t edgePhcNs)
{
    const int64_t nowUs = edgeNs / 1000;
    /* Stamped here rather than in the capture handler because this is the path
     * both handlers share: the capture peripheral is not always the one in use
     * (the GPIO fallback is what runs when the timer cannot be set up), and
     * anything waiting on "an edge arrived" was waiting on the wrong one. */
    /* Odd from here to the matching bump below: a task reading these words
     * without the lock sees the change and reads them again. */
    m_edgeSeq.fetch_add(1, std::memory_order_release);
    m_lastEdgeUs = nowUs;

    portENTER_CRITICAL_ISR(&m_mux);
    /* The POST /gps/pps hook swallows the edge here rather than further up, so
     * what it produces is the real holdover code path and not an imitation of
     * it. One extra comparison, against a word that is zero unless somebody
     * asked for the test. */
    if (nowUs >= m_ppsSuppressUntilUs &&
        (m_ppsCount == 0 || nowUs - m_ppsMonoUs >= kMinEdgeSpacingUs)) {
        m_ppsMonoUs = nowUs;
        m_ppsMonoNs = edgeNs;
        if (edgePhcNs != 0) {
            m_ppsPtpNs = edgePhcNs;
        }
        ++m_ppsCount;
    }
    portEXIT_CRITICAL_ISR(&m_mux);
    m_edgeSeq.fetch_add(1, std::memory_order_release);
}

/* The fallback edge handler, used only if the capture timer cannot be set up.
 * Stamps with esp_timer at whole-microsecond resolution, so the residual
 * carries the interrupt latency; pps_latency measures how much of it there
 * was. */
void IRAM_ATTR GpsTime::ppsIsr(void *arg)
{
    auto *self = static_cast<GpsTime *>(arg);
    /* esp_timer's counter is in IRAM (CONFIG_ESP_TIMER_IN_IRAM), which is what
     * lets this handler be an IRAM interrupt: an OTA write disables the flash
     * cache for milliseconds at a time, and an edge landing in that window must
     * not be a crash. */
    const int64_t stampUs = esp_timer_get_time();

    /* The same edge in the 1588 counter's base, read before the lock rather
     * than under it: the other core can hold m_mux for the length of a status
     * copy, and a read taken after that wait put the counter's anchor late by
     * the wait while the esp_timer stamp, taken on entry, was not. */
    const int64_t edgePhcNs = ptp::hardware() ? ptp::nowNs() : 0;

    /* After the stamp and the PHC read, so neither waits on the two timer
     * registers, and before recordEdge, which is where the anchor is set. */
    pps_latency::onEdge(stampUs);

    self->recordEdge(stampUs * 1000, edgePhcNs);
}

/* The edge as the capture peripheral latched it.
 *
 * The tick is latched by hardware at the edge, so no interrupt latency is in
 * the stamp. What is *not* available is a way to read the counter, so the
 * "now" end of the difference is taken by catching the same counter from
 * software and reading that latch. The interval between the two is a few ticks,
 * which is why the sub-microsecond part is finally real: the arithmetic happens
 * in esp_timer's own domain (now minus the elapsed ticks), not against a
 * foreign timebase. That is the difference from the two earlier attempts the
 * old comment describes -- neither systimer's raw count nor the CPU cycle
 * counter shares esp_timer's phase, but this does, because "now" is esp_timer.
 */
bool IRAM_ATTR GpsTime::ppsCapIsr(mcpwm_cap_channel_handle_t,
                                  const mcpwm_capture_event_data_t *edge, void *arg)
{
    auto         *self = static_cast<GpsTime *>(arg);
    const int64_t atUs = esp_timer_get_time();

    /* See ppsIsr: the PHC half of the anchor is taken before anything that can
     * wait. */
    const int64_t edgePhcNs = ptp::hardware() ? ptp::nowNs() : 0;

    /* The software catch below runs this handler again a tick or two later.
     * Real edges are a second apart, so anything this close is that echo. The
     * store itself is in recordEdge, which both handlers reach. */
    if (self->m_lastEdgeUs >= 0 && atUs - self->m_lastEdgeUs < kSoftCatchWindowUs) {
        return false;
    }

    int64_t edgeNs = atUs * 1000;
    if (self->m_capNowValid && self->m_capHz != 0) {
        /* Wrap-safe: one counter, one signed difference. */
        const uint32_t now =
            mcpwm_ll_timer_get_count_value(MCPWM_LL_GET_HW(kCapGroup), kCapNowTimer);
        const int32_t ticks = static_cast<int32_t>(now - edge->cap_value);
        if (ticks >= 0 && ticks <= kMaxCatchTicks) {
            edgeNs -= static_cast<int64_t>(ticks) * 1000000000LL / self->m_capHz;
        }
    }

    self->recordEdge(edgeNs, edgePhcNs);
    return false;
}

void GpsTime::taskEntry(void *arg)
{
    static_cast<GpsTime *>(arg)->task();
}

esp_err_t GpsTime::beginEdge()
{
    /* The pin is configured here and not with the UART, because the GPIO driver
     * fixes the core of its interrupt at the first pin whose interrupt is
     * enabled, and that has to be this call on the timing core. */
    gpio_config_t pps = {};
    pps.pin_bit_mask  = 1ull << kPpsPin;
    pps.mode          = GPIO_MODE_INPUT;
    pps.pull_up_en    = GPIO_PULLUP_DISABLE;
    /* Pulled down so an unconnected pin reads 0 rather than picking up noise;
     * the module drives the line push-pull, so the resistor never fights it. */
    pps.pull_down_en = GPIO_PULLDOWN_ENABLE;
    pps.intr_type    = GPIO_INTR_POSEDGE;
    esp_err_t err    = gpio_config(&pps);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d: %s -- no PPS", static_cast<int>(kPpsPin), esp_err_to_name(err));
        return err;
    }

    /* The latency probe is armed from app_main() once the link is up, so its
     * failure is readable over the network console. It measures the interrupt
     * stamp below; the capture path does not need it. */

    err = kCaptureEdge ? beginCapture() : ESP_ERR_NOT_SUPPORTED;
    if (err != ESP_OK) {
        /* The capture is how the edge is stamped without interrupt latency in
         * it, but a plain GPIO interrupt is a working second best and it is not
         * worth refusing to run over. */
        ESP_LOGW(TAG, "MCPWM capture unavailable (%s): falling back to an interrupt stamp",
                 esp_err_to_name(err));
        err = beginInterruptEdge();
    }
    return err;
}

/* An interrupt is bound to the core that installs it, and the installer cannot
 * be handed to esp_ipc: the GPIO driver makes its own IPC call to the same core
 * and would wait on the one it is running in. So a task pinned there does it
 * and goes away. */
esp_err_t GpsTime::beginEdgeOnTimingCore()
{
    struct Call {
        GpsTime          *self;
        TaskHandle_t      waiter;
        esp_err_t         err;
        std::atomic<bool> done;
    } call{this, xTaskGetCurrentTaskHandle(), ESP_FAIL, false};

    const auto run = [](void *arg) {
        auto *const        c      = static_cast<Call *>(arg);
        const TaskHandle_t waiter = c->waiter;
        c->err                    = c->self->beginEdge();
        /* Nothing of `c` is touched after this store: the waiter's frame is
         * free to go the moment it reads true. */
        c->done.store(true, std::memory_order_release);
        xTaskNotifyGive(waiter);
        vTaskDelete(nullptr);
    };
    if (xTaskCreatePinnedToCore(run, "pps_bind", 4096, &call, kTaskPriority, nullptr,
                                cores::kTiming) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    /* A notification from anywhere else must not end the wait early. */
    while (!call.done.load(std::memory_order_acquire)) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    return call.err;
}

/* The fallback: a GPIO interrupt that stamps on entry. Level 3, above the UART\'s
 * level 1 and the Ethernet\'s, so the edge nests over a serial or MAC handler
 * already running instead of waiting for it, and IRAM so it survives the flash
 * cache going away under an OTA write. Neither comes free: at level 3 the
 * handler may only use what is safe there. */
esp_err_t GpsTime::beginInterruptEdge()
{
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "GPIO ISR service already installed: the PPS handler may not be in IRAM");
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = gpio_isr_handler_add(kPpsPin, &GpsTime::ppsIsr, this);
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "PPS edge: GPIO interrupt on core %d (stamp on entry, latency included)",
                 static_cast<int>(xPortGetCoreID()));
    }
    return err;
}

/* The edge is latched by the capture peripheral at the edge itself, so the
 * stamp does not carry the interrupt latency, and the resolution is a tick of
 * the 80 MHz APB clock. The ISR only publishes the already-latched value.
 *
 * MCPWM needs its interrupt handler and its control functions in IRAM for this
 * to keep the old interrupt\'s property of running with the flash cache off
 * (CONFIG_MCPWM_ISR_CACHE_SAFE and CONFIG_MCPWM_CTRL_FUNC_IN_IRAM, both pinned in
 * sdkconfig.defaults), because the software catch used to read the counter is a
 * control call.
 */
esp_err_t GpsTime::beginCapture()
{
    mcpwm_capture_timer_config_t timerConfig = {};
    timerConfig.group_id                     = 0;
    timerConfig.clk_src                      = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
    esp_err_t err                            = mcpwm_new_capture_timer(&timerConfig, &m_capTimer);
    if (err != ESP_OK) {
        return err;
    }

    mcpwm_capture_channel_config_t channelConfig = {};
    channelConfig.gpio_num                       = kPpsPin;
    channelConfig.prescale                       = 1;
    channelConfig.flags.pos_edge                 = 1;
    channelConfig.flags.neg_edge                 = 0;

    err = mcpwm_new_capture_channel(m_capTimer, &channelConfig, &m_capChannel);
    if (err != ESP_OK) {
        return err;
    }

    mcpwm_capture_event_callbacks_t callbacks = {};
    callbacks.on_cap                          = &GpsTime::ppsCapIsr;
    err = mcpwm_capture_channel_register_event_callbacks(m_capChannel, &callbacks, this);
    if (err != ESP_OK) {
        return err;
    }
    err = mcpwm_capture_channel_enable(m_capChannel);
    if (err != ESP_OK) {
        return err;
    }
    err = mcpwm_capture_timer_enable(m_capTimer);
    if (err != ESP_OK) {
        return err;
    }
    err = mcpwm_capture_timer_start(m_capTimer);
    if (err != ESP_OK) {
        return err;
    }

    /* "Now" in the same clock. The capture timer has no readable counter, and
     * the software catch that would supply one carries jitter: as measured it
     * took the fit residual from 1.5 us to 4.3 us, worse than the latency the
     * capture removes. A PWM timer in the same group counts from the same clock
     * and costs one register read, so it is the "now" end -- after a check that
     * the two really do count at one rate. */
    mcpwm_ll_timer_set_count_mode(MCPWM_LL_GET_HW(kCapGroup), kCapNowTimer,
                                  MCPWM_TIMER_COUNT_MODE_UP);
    mcpwm_ll_timer_set_start_stop_command(MCPWM_LL_GET_HW(kCapGroup), kCapNowTimer,
                                          MCPWM_TIMER_START_NO_STOP);

    /* In Hz, not nanoseconds, and not the same on every part. */
    err = mcpwm_capture_timer_get_resolution(m_capTimer, &m_capHz);
    if (err != ESP_OK || m_capHz == 0) {
        ESP_LOGW(TAG, "capture resolution unknown: the stamp falls back to the interrupt time");
        m_capHz = 0;
    } else {
        ESP_LOGI(TAG,
                 "PPS edge: MCPWM capture at %lu Hz (%.1f ns a tick, latched at the edge, "
                 "no ISR latency)",
                 static_cast<unsigned long>(m_capHz), 1e9 / static_cast<double>(m_capHz));
    }

    /* Rate check: both counters are sampled across 10 ms through the same
     * jittery path, so jitter washes out and only a real clock difference shows.
     * The sub-microsecond part is dropped if they disagree. */
    uint32_t capFirst = 0, capLast = 0, nowFirst = 0, nowLast = 0;
    bool     sampled = false;
    if (mcpwm_capture_channel_trigger_soft_catch(m_capChannel) == ESP_OK &&
        mcpwm_capture_get_latched_value(m_capChannel, &capFirst) == ESP_OK) {
        nowFirst = mcpwm_ll_timer_get_count_value(MCPWM_LL_GET_HW(kCapGroup), kCapNowTimer);
        vTaskDelay(pdMS_TO_TICKS(10));
        if (mcpwm_capture_channel_trigger_soft_catch(m_capChannel) == ESP_OK &&
            mcpwm_capture_get_latched_value(m_capChannel, &capLast) == ESP_OK) {
            nowLast = mcpwm_ll_timer_get_count_value(MCPWM_LL_GET_HW(kCapGroup), kCapNowTimer);
            sampled = true;
        }
    }
    if (sampled) {
        const int32_t capRate = static_cast<int32_t>(capLast - capFirst);
        const int32_t nowRate = static_cast<int32_t>(nowLast - nowFirst);
        if (capRate > 0 && nowRate > 0) {
            const double ratio = static_cast<double>(nowRate) / static_cast<double>(capRate);
            m_capNowValid      = ratio > 0.99 && ratio < 1.01;
            ESP_LOGI(TAG, "capture counter vs group timer: %.4f%s", ratio,
                     m_capNowValid ? "" : " -- sub-microsecond stamp disabled");
        }
    }
    return ESP_OK;
}

esp_err_t GpsTime::begin()
{
    if (m_begun) {
        return ESP_OK;
    }

    /* Start at the rate this receiver was last heard on rather than at the
     * head of the list. The module keeps its own rate across a power cut, so
     * assuming the head means every boot walks the scan while the receiver is
     * still starting -- and a boot that arrives here with the module slow to
     * talk loses the correct rate and takes minutes to come back to it. */
    const std::string stored = Settings::instance().gpsBaud();
    m_preferredIndex         = 0;
    for (int i = 0; i < static_cast<int>(sizeof(kBauds) / sizeof(kBauds[0])); ++i) {
        if (stored == std::to_string(kBauds[i])) {
            m_preferredIndex = i;
            break;
        }
    }
    m_baudIndex = m_preferredIndex;

    uart_config_t config       = {};
    config.baud_rate           = static_cast<int>(kBauds[m_preferredIndex]);
    config.data_bits           = UART_DATA_8_BITS;
    config.parity              = UART_PARITY_DISABLE;
    config.stop_bits           = UART_STOP_BITS_1;
    config.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
    config.rx_flow_ctrl_thresh = 0;
    config.source_clk          = UART_SCLK_DEFAULT;

    /* A transmitter buffer is needed even though nothing is sent after boot: the
     * configuration messages go out through it. */
    esp_err_t err = uart_driver_install(kUart, 1024, 256, 0, nullptr, 0);
    if (err == ESP_OK) {
        err = uart_param_config(kUart, &config);
    }
    if (err == ESP_OK) {
        err = uart_set_pin(kUart, kTxPin, kRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART%d: %s -- no GPS", kUart, esp_err_to_name(err));
        return err;
    }

    err = beginEdgeOnTimingCore();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PPS edge on GPIO%d: %s", static_cast<int>(kPpsPin), esp_err_to_name(err));
        return err;
    }

    if (xTaskCreatePinnedToCore(&GpsTime::taskEntry, "gps", kTaskStack, this, kTaskPriority,
                                nullptr, cores::kWork) != pdPASS) {
        ESP_LOGE(TAG, "cannot start the GPS task");
        return ESP_ERR_NO_MEM;
    }

    /* The bias switch. Output, idle high (off, which is also the board's own
     * default), then turned on below; the sense pin is an input with the board's
     * pull-up already on the net. */
    gpio_config_t bias = {};
    bias.pin_bit_mask  = (1ULL << kAntDrivePin) | (1ULL << kAntSensePin);
    bias.mode          = GPIO_MODE_INPUT_OUTPUT;
    bias.pull_up_en    = GPIO_PULLUP_ENABLE;
    bias.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    bias.intr_type     = GPIO_INTR_DISABLE;
    if (gpio_config(&bias) != ESP_OK) {
        ESP_LOGW(TAG, "cannot claim GPIO%d/%d for the bias switch", kAntDrivePin, kAntSensePin);
    } else {
        gpio_set_direction(static_cast<gpio_num_t>(kAntDrivePin), GPIO_MODE_OUTPUT);
        gpio_set_direction(static_cast<gpio_num_t>(kAntSensePin), GPIO_MODE_INPUT);
        gpio_set_level(static_cast<gpio_num_t>(kAntDrivePin), 1); /* off until the module is up */
        ESP_LOGI(TAG, "antenna bias on GPIO%d (active low), short sense on GPIO%d", kAntDrivePin,
                 kAntSensePin);
    }

    if (xTaskCreatePinnedToCore(&GpsTime::bridgeTaskEntry, "gpsbridge", 4096, this, 3, nullptr,
                                cores::kWork) != pdPASS) {
        ESP_LOGW(TAG, "no bridge task: u-center passthrough unavailable");
    }

    /* The decoder emits into this object; without the listener it counts frames
     * and dispatches none of them, which reads as a receiver sending nothing
     * useful. */
    m_decoder.listen(this);
    m_begun       = true;
    m_baud        = kBauds[m_preferredIndex];
    m_nextScanUs  = esp_timer_get_time() + kFirstRateUs;
    m_nextCheckUs = esp_timer_get_time() + 1000000;

    /* A lone TIM-TP enable three seconds in, if none has produced a frame. Ask
     * it on its own, with nothing else outstanding, so the ACK or NAK that comes
     * back can only be about this one message -- in the boot burst the replies
     * to eight identical CFG-MSG frames are indistinguishable. Doubles as the
     * retry for a module that was still booting when the burst went out. */
    m_timTpProbeAtUs = esp_timer_get_time() + 3000000;
    m_nextHwUs       = esp_timer_get_time() + 2000000;
    m_warmRetryAtUs  = esp_timer_get_time() + 10000000;

    ESP_LOGI(TAG, "UART%d %u 8N1 (esp TX %d -> module RX, esp RX %d <- module TX), PPS on GPIO%d",
             kUart, static_cast<unsigned>(m_baud), kTxPin, kRxPin, static_cast<int>(kPpsPin));
    return ESP_OK;
}

void GpsTime::task()
{
    uint8_t buf[64];

    /* Ask before listening: a receiver still in its factory NMEA mode is using
     * most of the line, and switching that off is the fastest way to
     * make room for the messages this reference actually reads. Harmless noise
     * if there is nothing on the other end. */
    configureModule();
    restoreTimeMode();

    /* Power the antenna once the receiver has been configured. Off during the
     * baud scan so a wrong lead is not sitting powered through it. */
    vTaskDelay(pdMS_TO_TICKS(1500));
    applyBias(true);
    ESP_LOGI(TAG, "antenna bias on");

    for (;;) {
        /* While u-center is attached, the bridge task does the reading and taps
         * the bytes into the parser, so only the read is skipped here. The
         * periodic work below carries on: the clock must keep being disciplined
         * and /healthz must keep telling the truth while somebody is poking at
         * the receiver.
         *
         * Only the read is contended, not the parser -- two readers on one
         * stream is what the bridge exists to avoid. */
        if (m_bridgeActive) {
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            const int got = uart_read_bytes(kUart, buf, sizeof(buf), pdMS_TO_TICKS(100));
            m_decoder.feed(std::span(buf, static_cast<std::size_t>(got > 0 ? got : 0)));
        }

        /* TIM-TP is only produced while the receiver is surveying.
         *
         * This cost a morning. In fixed (time) mode the M8T acknowledges the
         * rate, answers a poll with nothing, and emits no frame at all -- for
         * hours, across reboots, a power reset and a configuration default
         * clear. Back in survey mode it returns at 1 Hz immediately (42 replies
         * against 0). NAV-TIM-TP, the same bytes under class 0x01, is never
         * answered on this part at all, so it is polled too rather than assumed.
         *
         * A poll is asked for once a second because the rate is not honoured
         * even when the message does flow. Sixteen bytes back; the reply arrives
         * as a normal frame and lands in onNavTimTp. The consequence worth
         * knowing: the pulse statement and qErr are available in the mode that
         * is *worse* for timing, and absent in the mode that is better. */
        const int64_t nowUs = esp_timer_get_time();
        /* The status is published here, once a NAV-PVT has been handled: that
         * is a quarter of a second after the edge and most of a second before
         * the next. With no frames arriving it still goes out, on the clock. */
        if (m_statusDue || nowUs - m_statusPublishedUs >= 1100000) {
            publishStatus();
        }
        if (m_timTpCfg == 1 && nowUs - m_timTpPolledUs >= 1000000) {
            m_timTpPolledUs = nowUs;
            sendUbx(0x0D, 0x01, nullptr, 0);
            sendUbx(0x01, 0x27, nullptr, 0);
        }

        /* The scan used to run only until the first frame of the boot, so a rate
         * set from the bridge -- or by anything else that moved the module --
         * wedged the line until the next power cycle. Silence now re-arms it. A
         * session on tcp/3333 holds the port and must not have the rate moved
         * under it, so the bridge suppresses the scan for its duration. */
        if (!m_bridgeActive && m_frameSeen && m_lastFrameUs != 0 &&
            nowUs - m_lastFrameUs > kFrameSilenceUs) {
            m_frameSeen   = false;
            m_lastFrameUs = 0;
            ++m_rescans;
            /* The rate that was working a moment ago gets its full turn before
             * the scan moves. The deadline used to be whatever the last scan
             * left behind -- on a healthy box, fifteen seconds after boot -- so
             * the first silent five seconds walked straight off the right rate
             * and began configuring the module blind at the wrong ones. A
             * decoder caught mid-frame is the likelier cause than a rate that
             * moved, and starting it again is the whole cure for that. */
            m_decoder.reset();
            m_nextScanUs = nowUs + kFirstRateUs;
            ESP_LOGW(TAG, "no frames for %lld ms: rescanning the baud rate",
                     static_cast<long long>(kFrameSilenceUs / 1000));
        }
        /* Frame rate, evaluated once a second. Three poor seconds in a row is
         * treated as the same condition the silence test uses, so the scan
         * advances. Without it a mismatched rate that still parses the odd
         * frame sits there indefinitely. */
        if (m_secStartUs == 0) {
            m_secStartUs = nowUs;
        } else if (nowUs - m_secStartUs >= 1000000) {
            /* No exemption while the config burst is talking. A rate that works
             * answers the burst with ACKs, so an empty second during a config is
             * itself evidence; a rate that does not work sits there silently. The
             * exemption used to be unconditional, and it was what wedged the boot
             * at 9600: the burst runs every six seconds, so it kept zeroing the
             * counter and the poor-rate test could never reach its limit. */
            if (m_secFrames < kMinFramesPerSec) {
                if (m_poorSecs < 255) {
                    ++m_poorSecs;
                }
                if (m_poorSecs >= 6) {
                    m_frameSeen = false;
                    m_poorSecs  = 0;
                    m_decoder.reset();
                    ESP_LOGW(TAG, "only %u frames in 6 s at %u baud: rescanning",
                             static_cast<unsigned>(m_secFrames * 6), static_cast<unsigned>(m_baud));
                }
            } else {
                m_poorSecs = 0;
            }
            m_secFrames  = 0;
            m_secStartUs = nowUs;
        }

        if (!m_frameSeen && nowUs >= m_nextScanUs) {
            advanceBaud();
            m_nextScanUs = nowUs + (m_baudIndex == m_preferredIndex ? kFirstRateUs : kScanAfterUs);
        }
        if (nowUs >= m_nextCheckUs) {
            logLockChange();
            m_nextCheckUs = nowUs + 1000000;

            /* Nothing heard for a while: is the receiver transmitting at all?
             * The RX pin is sampled for transitions rather than the UART being
             * asked, because the question is below the driver -- a pin that
             * never moves is a wire or a dead port, a pin that moves while no
             * frame parses is a rate or framing fault, and the two need
             * different work. */
            if (!m_frameSeen) {
                int edges = 0;
                int last  = gpio_get_level(static_cast<gpio_num_t>(kRxPin));
                for (int i = 0; i < 400; ++i) {
                    const int now = gpio_get_level(static_cast<gpio_num_t>(kRxPin));
                    if (now != last) {
                        ++edges;
                        last = now;
                    }
                    esp_rom_delay_us(500);
                }
                ESP_LOGW(TAG,
                         "GPIO%d idle probe at %u baud: %d transitions in 200 ms "
                         "(level %d)",
                         kRxPin, static_cast<unsigned>(m_baud), edges, last);
            }
        }
        if (m_baudReq != 0) {
            const uint32_t want   = m_baudReq;
            const bool     module = m_baudReqModule;
            m_baudReq             = 0;
            m_baudReqModule       = false;
            if (module) {
                /* Tell the receiver first, while the host is still listening at
                 * a rate the receiver knows. Then hold the old rate: if frames
                 * stop, this is written back as soon as anything is heard
                 * again, on whatever rate that turns out to be. */
                m_baudPrev = m_baud;
                /* Four silence windows, not two. This module can take most of
                 * ten seconds to answer at a new rate -- measured: it accepted
                 * 115200, sent frames at 8.7 s, and had already been reverted at
                 * 8.0 s, so the change looked like a failure and undid itself. */
                m_baudRevertAtUs    = esp_timer_get_time() + 4 * kFrameSilenceUs;
                m_baudRevertPending = true;
                uart_wait_tx_done(kUart, pdMS_TO_TICKS(50));
                /* Acknowledged, not fire-and-forget: saving while the port
                 * change is still in flight captures the OLD rate, and the save
                 * itself reloads the port, so the change silently undoes. */
                /* This module does not ACK CFG-PRT -- it goes unanswered even
                 * when the change takes -- so silence has to mean "carry on".
                 * An explicit NAK is the only thing that stops the move, and
                 * the revert covers a change that simply does not work. */
                const int8_t prt = sendCfgPrt(want);
                if (prt == 0) {
                    ESP_LOGW(TAG, "CFG-PRT rejected: rate unchanged");
                } else {
                    /* Saved so the next power cycle comes back at the new rate
                     * rather than at 9600, which would undo the whole move. The
                     * pause lets the port change land before it is captured. */
                    uart_wait_tx_done(kUart, pdMS_TO_TICKS(50));
                    esp_rom_delay_us(200000);
                    const int8_t saved = saveCfgIo();
                    ESP_LOGW(TAG, "port set to %u and %s", static_cast<unsigned>(want),
                             saved == 1   ? "saved"
                             : saved == 0 ? "save REJECTED"
                                          : "save unanswered");
                }
                uart_wait_tx_done(kUart, pdMS_TO_TICKS(100));
            }
            uart_set_baudrate(kUart, want);
            m_baud      = want;
            m_frameSeen = false;
            ++m_rescans;
            /* The new rate is listened to until the revert deadline, or for a
             * first-rate turn when only the host moved. The scan deadline was
             * left stale here, so the very next pass walked off the rate that
             * had just been asked for. */
            m_decoder.reset();
            m_nextScanUs = module ? m_baudRevertAtUs : nowUs + kFirstRateUs;
            for (int i = 0; i < static_cast<int>(sizeof(kBauds) / sizeof(kBauds[0])); ++i) {
                if (kBauds[i] == want) {
                    m_baudIndex = i;
                }
            }
            if (module) {
                ESP_LOGW(TAG,
                         "receiver and host set to %u baud; reverting to %u if frames do not "
                         "resume",
                         static_cast<unsigned>(want), static_cast<unsigned>(m_baudPrev));
            } else {
                ESP_LOGW(TAG, "host side set to %u baud alone: the scan will find the receiver",
                         static_cast<unsigned>(want));
            }
        }

        /* The revert. Either frames came back and the old rate is written to the
         * receiver again, or the wait expires and the next rate the scan lands
         * on is treated as the one to write it at. */
        /* Frames at the new rate: the move took, and there is nothing to put
         * back. The old rate used to be armed for write-back at the request, so
         * the first frame at the new rate -- success -- was what undid it. */
        if (m_baudRevertPending && m_frameSeen) {
            m_baudRevertPending = false;
        }
        if (m_baudRevertPending && nowUs >= m_baudRevertAtUs) {
            m_baudRevertPending = false;
            m_baudRevertTo      = m_baudPrev;
            ESP_LOGW(TAG, "no frames at %u baud: reverting to %u", static_cast<unsigned>(m_baud),
                     static_cast<unsigned>(m_baudPrev));
        }
        if (m_baudRevertTo != 0 && m_frameSeen) {
            const uint32_t back = m_baudRevertTo;
            m_baudRevertTo      = 0;
            uart_wait_tx_done(kUart, pdMS_TO_TICKS(50));
            /* Saved as well, and for the same reason as the forward path: the
             * forward change already saved ITS rate, so reverting only in RAM
             * would leave the rate that failed in flash -- and the next power
             * cycle would come straight back to it. */
            if (sendCfgPrt(back) != 0) {
                uart_wait_tx_done(kUart, pdMS_TO_TICKS(50));
                esp_rom_delay_us(200000);
                saveCfgIo();
            }
            uart_wait_tx_done(kUart, pdMS_TO_TICKS(100));
            uart_set_baudrate(kUart, back);
            m_baud              = back;
            m_baudRevertPending = false;
            ESP_LOGW(TAG, "reverted to %u baud", static_cast<unsigned>(back));
        }
        /* A bias change or a restart asked for over HTTP. Serviced here rather
         * than in the handler so exactly one thread owns the UART and the
         * parser. */
        if (m_biasReq >= 0) {
            const bool on    = m_biasReq == 1;
            const bool wasOn = m_antBiasOn;
            m_biasReq        = -1;
            /* Supply control bit on or off; short/open detection and
             * power-down-on-short stay set either way. */
            const uint8_t cfg[4] = {static_cast<uint8_t>(on ? 0x0F : 0x0E), 0, 0, 0};
            const int8_t  br     = sendAndWait(0x06, 0x13, cfg, sizeof(cfg));
            /* The pin is what actually does it; the CFG-ANT frame is kept
             * because it costs nothing and would start working on a module that
             * has a supervisor. */
            m_antLatched   = false;
            m_shortSamples = 0;
            applyBias(on);
            gpio_set_level(static_cast<gpio_num_t>(kAntDrivePin), on ? 0 : 1);
            ESP_LOGW(TAG, "antenna bias %s on request (module said %s)", on ? "ON" : "OFF",
                     br == 1   ? "accepted"
                     : br == 0 ? "refused"
                               : "nothing");
            sendUbx(0x0A, 0x09, nullptr, 0); /* re-read the supervisor at once */

            /* Powering the preamp back up is not enough to get the sky back. The
             * supply was restored and the receiver sat at zero satellites, a
             * pegged AGC and a DONTKNOW antenna status for four minutes; the
             * same supply at the next boot works, so what is missing is the
             * module re-initialising its front end. A hardware reset gives it
             * that, and it keeps the almanac. Only on the off -> on edge: an
             * "on" while already on has nothing to re-initialise. */
            if (on && !wasOn) {
                m_resetReq   = true;
                m_resetHard  = true;
                m_resetCold  = false;
                m_resetPower = false;
                ESP_LOGW(TAG, "hardware reset queued to re-initialise the front end");
            }
        }
        if (m_gnssReq) {
            m_gnssReq  = false;
            m_gnssMask = m_gnssReqMask;
            /* The whole message set, not one frame. A mask change throws the
             * acquisition away, and configureModule() is the only thing that
             * knows every message the clock needs, so letting it run is
             * cheaper than a second path that could drift from it. */
            ESP_LOGW(TAG, "constellation mask 0x%02x requested", static_cast<unsigned>(m_gnssMask));
            configureModule();
        }
        if (m_svinCmd != 0) {
            const uint8_t cmd = m_svinCmd;
            m_svinCmd         = 0;
            /* The command names what only the task may do -- touch the UART --
             * so the payload is built here rather than by the handler.
             *
             * **CFG-TMODE2 (0x06 0x3D), not TMODE3 (0x06 0x71).** At protocol
             * 22 this is an M8, and TMODE3 is not its message. What gave the
             * mistake away is that TMODE3 was acknowledged and then read back
             * as all zeros: the same 28 bytes under TMODE2's layout start with
             * a byte that is `version` in one and `timeMode` in the other, so
             * TMODE3 was being told "disabled" each time. NAV-SVIN being NAKed
             * while TIM-SVIN (0x0D 0x04) answers says the same about the status
             * message -- this is a TIM firmware and the TIM dialect is the one
             * it speaks. */
            uint8_t  tm[28]   = {};
            uint32_t fixedAcc = 0;
            uint32_t minDur   = 0;
            uint32_t accLimit = 0;
            /* A survey reply is the only thing that fills the mean, so "fix" on a
             * box whose position came back from NVS has no mean at all. Locking
             * the (0,0,0) that is left points the receiver at the centre of the
             * Earth, and kilometres of position are microseconds of clock: the
             * box served 10 ms out until it was stopped. The held position is
             * the right thing to re-lock from in that case, and only when there
             * is neither is there nothing to lock. */
            const bool haveMean = !(m_svinMean[0] == 0 && m_svinMean[1] == 0 && m_svinMean[2] == 0);
            if (cmd == 3 && !haveMean && !m_fixedValid) {
                ESP_LOGW(TAG, "nothing to lock: no survey mean and no held position, "
                              "position left moving");
                m_svinCmd = 0;
            } else if (cmd == 1) {
                tm[0]  = 1; /* survey-in */
                minDur = m_svinDurS;
                /* Millimetres, not millimetres squared. Both accuracy fields
                 * in this message are plain mm with no scaling (spec R27 p.267);
                 * everything the receiver says about its survey is a variance in
                 * mm squared, so the square has to come off here. It used to go
                 * on the wire squared: a 30 m limit was sent as 9e8 mm, i.e.
                 * 900 km, which no site can fail. */
                accLimit = m_svinAccMm;
            } else if (cmd == 3) {
                /* ECEF from the receiver's own mean: it reports ECEF and takes
                 * ECEF, so staying in it avoids a projection between two things
                 * that already agree. The accuracy is the mean's own standard
                 * deviation, again in mm out of a variance in mm squared. This
                 * is not decoration: the spec (R27 p.100) says the 3D accuracy
                 * here "directly affects the accuracy of the timepulse", so a
                 * variance of 1.4e7 sent raw told the receiver its position was
                 * good to 14 000 km and put the stated pulse accuracy near 47 ms
                 * -- which is the size of the errors that then went unnoticed. */
                tm[0] = 2; /* fixed */
                /* Re-locking what is already held keeps its accuracy and its
                 * provenance; only a mean from this survey can supersede it. */
                fixedAcc = haveMean ? isqrt(m_svinAcc) : m_fixedAcc;
                for (int i = 0; i < 3; ++i) {
                    text::putLe(tm + 4 + 4 * i, haveMean ? m_svinMean[i] : m_fixedMean[i]);
                }
            }
            /* flags stay zero: ECEF, not LLA. */
            text::putLe(tm + 16, fixedAcc);
            text::putLe(tm + 20, minDur);
            text::putLe(tm + 24, accLimit);
            const int8_t r = sendAndWait(0x06, 0x3D, tm, sizeof(tm));
            if (r == 1) {
                m_tmode      = cmd == 1 ? 1 : cmd == 3 ? 2 : 0;
                m_holdWanted = cmd == 3;
                /* The two persisted forms, read once into plain values: the
                 * members are volatile, and a volatile reference is not one of
                 * the types std::formatter is specialised for. */
                if (cmd == 1) {
                    const auto hours = static_cast<uint32_t>(m_svinDurS / 3600);
                    const auto accMm = static_cast<uint32_t>(m_svinAccMm);
                    Settings::instance().setGpsSurvey(std::format("{},{}", hours, accMm));
                }
                if (cmd == 3) {
                    /* Set before the write, because the write is what has to
                     * carry it. */
                    if (haveMean) {
                        portENTER_CRITICAL(&m_mux);
                        m_holdValidated = m_svinValid;
                        portEXIT_CRITICAL(&m_mux);
                    }
                    Settings::instance().setGpsFixedPos(std::format(
                        "{},{},{},{},{}", haveMean ? m_svinMean[0] : m_fixedMean[0],
                        haveMean ? m_svinMean[1] : m_fixedMean[1],
                        haveMean ? m_svinMean[2] : m_fixedMean[2],
                        /* mm, like the wire: this stored the
                                     * variance, and the boot re-sent it as an
                                     * accuracy of kilometres. */
                        haveMean ? isqrt(m_svinAcc) : m_fixedAcc, m_holdValidated ? 1 : 0));
                }
                saveTimeMode();
                m_svinAtUs = 0;
                if (cmd == 1) {
                    ESP_LOGW(TAG, "survey-in started: %u s minimum, %u mm limit",
                             static_cast<unsigned>(minDur), static_cast<unsigned>(m_svinAccMm));
                    /* The limit is compared against the site's wander, so one
                     * below what the receiver is already scattering can never
                     * be met and the survey will run the duration out. Say so
                     * at the start rather than 24 hours later. */
                    if (m_svinAcc != 0 && isqrt(m_svinAcc) > accLimit) {
                        ESP_LOGW(TAG,
                                 "  the limit is tighter than this site scatters (%u mm "
                                 "now): expect it to run the full duration",
                                 static_cast<unsigned>(isqrt(m_svinAcc)));
                    }
                } else if (cmd == 3) {
                    ESP_LOGW(TAG, "position locked fixed at %d %d %d mm", m_svinMean[0],
                             m_svinMean[1], m_svinMean[2]);
                    /* A lock taken before the survey signed itself off is a
                     * position the receiver never agreed was good enough. It is
                     * usually fine -- metres are nanoseconds -- but it is also
                     * the state that produced the 18:43 free-run, and nothing
                     * said so at the time. Say it, and keep saying it in
                     * /gps/info through hold_validated. */
                    if (!m_holdValidated) {
                        ESP_LOGW(TAG,
                                 "  the survey had NOT met its own limit (%u mm needed, "
                                 "%u mm measured): locked unvalidated",
                                 static_cast<unsigned>(m_svinAccMm),
                                 static_cast<unsigned>(isqrt(m_svinAcc)));
                    }
                } else {
                    ESP_LOGW(TAG, "time mode cleared: back to a moving position");
                }
            } else {
                ESP_LOGW(TAG, "CFG-TMODE2 %s: time mode unchanged",
                         r == 0 ? "rejected" : "unanswered");
            }
        }
        /* A new cable or user delay re-sends the pulse frame and nothing else.
         * The survey and the mask are untouched: this is a property of the lead,
         * and restarting a day's survey to change it would be a poor trade.
         *
         * The rate window is thrown away, though, and that is the point. A delay
         * is a step in the phase: inside a 300 s fit a 10 ms step reads as
         * 33 ppm of rate error, so the servo trims it back out over the window
         * and the correction quietly undoes itself. Measured: a 10 ms delay
         * moved the pulse for a second or two and left the served time where it
         * was. With the window cleared, the step is geometry again. */
        if (m_tp5Cmd) {
            m_tp5Cmd       = false;
            const int8_t r = sendTp5();
            m_tp5Acked     = r == 1;
            m_tp5Naked     = r == 0;
            if (r == 1) {
                m_rateCount         = 0;
                m_rateNext          = 0;
                m_rateLogged        = false;
                m_lastRateOffsetSet = false;
            } else {
                ESP_LOGW(TAG, "CFG-TP5 delay change %s", r == 0 ? "rejected" : "unanswered");
            }
        }
        /* Three more tries at the two messages a module still booting may have
         * refused, ten seconds apart. The boot burst goes out about half a
         * second after power-up, which on this receiver is before it is ready
         * for everything -- MON-VER only answered at 1.2 s -- so a NAK that
         * early is not always a refusal. Only the refusals are retried; a
         * message that was acknowledged is left alone. */
        if (m_warmRetries < 3 && nowUs >= m_warmRetryAtUs && (m_antCfgNaked || m_timTpNaked)) {
            ++m_warmRetries;
            m_warmRetryAtUs = nowUs + 10000000;
            if (m_antCfgNaked) {
                const uint8_t cfg[4] = {0x0B, 0x00, 20, 27};
                const int8_t  br     = sendAndWait(0x06, 0x13, cfg, sizeof(cfg));
                if (br >= 0) {
                    m_antCfgNaked = br == 0;
                    m_antCfgAcked = br == 1;
                    ESP_LOGW(TAG, "CFG-ANT retry %u: %s", m_warmRetries,
                             br == 1 ? "accepted now the receiver is up" : "still refused");
                }
            }
            if (m_timTpNaked && m_timTpCount == 0) {
                const int8_t br = sendAndWait(0x06, 0x01, kTimTpEnable, sizeof(kTimTpEnable));
                if (br >= 0) {
                    m_timTpNaked = br == 0;
                    m_timTpAcked = br == 1;
                    ESP_LOGW(TAG, "TIM-TP retry %u: %s", m_warmRetries,
                             br == 1 ? "accepted now the receiver is up" : "still refused");
                }
            }
        }

        if (m_reconfigAtUs != 0 && nowUs >= m_reconfigAtUs) {
            if (m_bridgeActive && !m_reconfigForce) {
                /* A client is configuring the receiver. Stamping eleven CFG-MSG
                 * frames of our own over the top of whatever it set is how a
                 * session ends up fighting the firmware for the module. Stand
                 * aside -- detaching re-arms this anyway. */
                m_reconfigAtUs = 0;
            } else {
                const bool forced = m_reconfigForce;
                m_reconfigAtUs    = 0;
                m_reconfigForce   = false;
                m_skipAdopt       = forced;
                configureModule();
                m_skipAdopt = false;
                if (forced) {
                    /* A reset or a clear-to-defaults took the time mode with it.
                     * Put a known position hold back, and do not ask the module
                     * what its mode is first: it is still rebooting and would
                     * answer with the mode the reset left behind. */
                    reassertFix();
                }
            }
        }
        /* The hold is asserted again if the receiver is not holding and the
         * settings still want it. The stored mode is read rather than a member,
         * because "stop" puts the module back to moving and only the settings
         * know that has happened -- without this test the retry would put a hold
         * back under a box that had been asked for a moving solution. A survey
         * in progress wins for the same reason: TMODE2 fixed would end it.
         *
         * It also waits for a fix, satellites and a live edge, because the frame
         * restarts the pulse generator and there is nothing to restart before
         * the receiver has begun: an unfixed module given TMODE2 spends the
         * boot's first edges and can hand the clock to the fallback. Waiting
         * does not cost a retry, so the deadline is simply moved on. */
        if (m_holdAssertAtUs != 0 && m_tmode == 2 && !m_svinActive) {
            /* The receiver is already holding -- the poll said so at boot, or a
             * previous attempt took -- so nothing is outstanding. Left armed the
             * deadline reads as overdue forever in /gps/info. */
            m_holdAssertAtUs = 0;
        } else if (m_holdWanted && m_tmode != 2 && m_holdAssertAtUs != 0 && !m_svinActive &&
                   Settings::instance().gpsTimeMode() == "fixed") {
            const bool ready = m_fixType >= 2 && m_satCount >= 4 && m_lastEdgeUs > 0 &&
                               nowUs - m_lastEdgeUs < 3000000;
            if (nowUs < m_holdAssertAtUs) {
                /* Not due yet. */
            } else if (!ready) {
                m_holdAssertAtUs = nowUs + kHoldRetryUs;
            } else if (sendFixedPosition()) {
                m_holdAssertAtUs = 0;
                m_holdRetryUs    = 0;
                ESP_LOGW(TAG, "position hold asserted at the receiver");
            } else {
                m_holdRetryUs    = std::min(m_holdRetryUs * 2, kHoldRetryMaxUs);
                m_holdAssertAtUs = nowUs + m_holdRetryUs;
                ESP_LOGW(TAG, "receiver refused the position hold: next try in %lld s",
                         static_cast<long long>(m_holdRetryUs / 1000000));
            }
        }
        if (m_defaultsReq) {
            m_defaultsReq = false;
            /* CFG-CFG with clearMask 0x1F: put every configuration block back to
             * its default in RAM. Unlike the resets, this touches the settings
             * themselves -- constellation selection, message rates, the lot --
             * and leaves the module's saved flash alone, so u-center still gets
             * its defaults back at the next power-up. The last software lever
             * when the receiver is deaf and its configuration is the suspect. */
            const uint8_t cfg[12] = {0x1F, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            ESP_LOGW(TAG, "receiver configuration cleared to defaults");
            sendUbx(0x06, 0x09, cfg, sizeof(cfg));
            /* The clear takes CFG-PRT back to the module's 9600 default with it.
             * Left alone the scan adopts 9600, then walks down looking for a
             * line that will carry the message set, and at 4800 the frames are
             * late enough to spike an edge a minute -- 109 of them and 1.5 ms of
             * offset before this was spotted. Put the working rate straight back
             * rather than let the scan find its way there. */
            requestBaud(m_baud, true);
            /* Forced: the clear took every rate to zero, so a client attached
             * right now has no stream either. */
            m_reconfigForce = true;
            m_reconfigAtUs  = esp_timer_get_time() + 3000000;
        }
        if (m_resetReq) {
            m_resetReq = false;
            /* CFG-RST. Hot by default: restart the navigation engine and keep
             * the almanac, which is the cheap way back from a state a module will
             * not leave on its own. Cold when asked, which throws the almanac
             * away -- the thing to try when a hot start changed nothing and the
             * receiver's own stored state is suspect. */
            const bool cold  = m_resetCold;
            const bool hard  = m_resetHard;
            const bool power = m_resetPower;
            m_resetHard      = false;
            m_resetPower     = false;
            /* resetMode: 0x00 is a hardware reset, 0x02 a controlled software
             * one (GNSS only). A hardware reset also drops the module's whole
             * runtime configuration back to its defaults, so the message set is
             * re-sent a few seconds later -- otherwise it comes back speaking
             * NMEA at 9600 and the parser has nothing to read. */
            /* 0x04 is "hardware reset after shutdown", the nearest thing to
             * pulling the module's supply: it shuts the receiver down first, so
             * the RF and baseband are re-initialised rather than restarted, and
             * every RAM configuration written since power-up goes with it. The
             * saved configuration in the module's own flash comes back, which is
             * the point when the RAM state is what is suspect. */
            const uint8_t mode   = power ? 0x04 : hard ? 0x00 : 0x02;
            const uint8_t rst[4] = {static_cast<uint8_t>(cold ? 0xFF : 0x00),
                                    static_cast<uint8_t>(cold ? 0xFF : 0x00), mode, 0x00};
            ESP_LOGW(TAG, "receiver %s%s start requested",
                     power  ? "POWER-CYCLE "
                     : hard ? "HARDWARE "
                            : "",
                     cold ? "cold" : "hot");
            if (hard || power) {
                /* Forced for the same reason as the clear above: the module
                 * comes back at its own defaults, in NMEA, at 9600, and a
                 * client attached through the reset needs a UBX stream just as
                 * much as the clock does. */
                m_reconfigForce = true;
                m_reconfigAtUs  = esp_timer_get_time() + 3000000;
            }
            sendUbx(0x06, 0x04, rst, sizeof(rst));
        }

        /* The short sense. Sample four times a second; three consecutive reads
         * in the shorted state cut the bias and latch, because the point of the
         * protection is to act faster than a person. */
        {
            static int64_t nextSenseUs = 0;
            if (nowUs >= nextSenseUs) {
                nextSenseUs     = nowUs + 250000;
                const int level = gpio_get_level(static_cast<gpio_num_t>(kAntSensePin));
                if (level != m_senseLogged) {
                    m_senseLogged = level;
                    ESP_LOGI(TAG, "antenna sense GPIO%d reads %d", kAntSensePin, level);
                }
                /* SCS1_N is active low: a short pulls it down. */
                if (m_antBiasOn && !m_antLatched && level == 0) {
                    if (++m_shortSamples >= 3) {
                        m_antShort   = true;
                        m_antLatched = true;
                        applyBias(false);
                        ESP_LOGW(TAG, "antenna SHORT: bias cut and latched off");
                    }
                } else if (level != 0) {
                    m_shortSamples = 0;
                    m_antShort     = false;
                }
            }
        }

        /* Antenna supervisor, every two seconds. Polled rather than enabled at
         * a rate so a bench session that never looks at it costs nothing. */
        if (!m_bridgeActive && nowUs >= m_nextHwUs) {
            sendUbx(0x0A, 0x09, nullptr, 0);
            m_nextHwUs = nowUs + 2000000;
        }
        if (!m_timTpProbed && m_timTpCount == 0 && nowUs >= m_timTpProbeAtUs) {
            m_timTpProbed   = true;
            const int8_t pr = sendAndWait(0x06, 0x01, kTimTpEnable, sizeof(kTimTpEnable));
            if (pr == 0) {
                m_timTpNaked = true;
                m_timTpAcked = false;
            } else if (pr == 1) {
                m_timTpAcked = true;
            }
        }
    }
}

uint32_t GpsTime::suppressPps(uint32_t seconds)
{
    const int64_t until =
        seconds == 0 ? 0 : esp_timer_get_time() + static_cast<int64_t>(seconds) * 1000000;

    portENTER_CRITICAL(&m_mux);
    m_ppsSuppressUntilUs = until;
    portEXIT_CRITICAL(&m_mux);

    if (seconds == 0) {
        ESP_LOGW(TAG, "PPS edges counted again: the holdover test is over");
        return 0;
    }

    const Status st = status();
    ESP_LOGW(TAG,
             "PPS edges ignored for %u s at the request of POST /gps/pps: the holdover from "
             "here is deliberate, and %s",
             static_cast<unsigned>(seconds),
             st.rateSamples >= kRateMinSamples ? "the servo is carrying it"
                                               : "with no rate yet it demotes in 3 s");
    return seconds;
}
