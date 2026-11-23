#include "gps/gps_internal.h"

#include "gps/mono.h"
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

/* Bench setting, not persisted. */
void GpsTime::requestPulseMs(uint32_t ms)
{
    m_pulseMs = ms;
    m_tp5Cmd  = true;
}

/* Bench setting, not persisted; 1000000 is 1 Hz. */
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
    /* m_edgeSeq is odd while these words change; readEdge() retries on it. */
    m_edgeSeq.fetch_add(1, std::memory_order_release);
    m_lastEdgeUs = nowUs;

    portENTER_CRITICAL_ISR(&m_mux);
    /* The POST /gps/pps test drops edges here, so holdover runs the real path. */
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

/* The GPIO edge handler, in use while kCaptureEdge is off. The stamp is the
 * systimer read at 62.5 ns; pps_latency takes the interrupt wait back out. */
void IRAM_ATTR GpsTime::ppsIsr(void *arg)
{
    auto *self = static_cast<GpsTime *>(arg);
    /* The systimer and the PHC are both register reads, so this runs with
     * the flash cache off during an OTA write. */
    pps_latency::beforeStamp();
    const int64_t stampNs = mono::nowNs();
    pps_latency::afterStamp();

    /* Read before taking m_mux, which the other core may be holding. */
    const int64_t edgePhcNs = ptp::hardware() ? ptp::nowNs() : 0;
    pps_latency::afterPhc();

    /* Core 1 is occasionally held off for 2-10 us. A wait before a read is
     * subtracted; a stall inside one cannot be placed, so the edge is dropped. */
    const pps_latency::Edge edge = pps_latency::finish(stampNs);
    if (!edge.placed) {
        return;
    }
    self->recordEdge(edge.edgeNs, edgePhcNs != 0 ? edgePhcNs - edge.phcLateNs : 0);
}

/* The MCPWM capture handler (kCaptureEdge). The edge tick is latched by
 * hardware; the stamp is mono now, less the ticks since the edge as read from
 * the group's PWM timer. */
bool IRAM_ATTR GpsTime::ppsCapIsr(mcpwm_cap_channel_handle_t,
                                  const mcpwm_capture_event_data_t *edge, void *arg)
{
    auto         *self = static_cast<GpsTime *>(arg);
    const int64_t atNs = mono::nowNs();

    /* See ppsIsr. */
    const int64_t edgePhcNs = ptp::hardware() ? ptp::nowNs() : 0;

    /* A software catch re-enters this handler moments after the edge. */
    if (self->m_lastEdgeUs >= 0 && atNs / 1000 - self->m_lastEdgeUs <
                                      kSoftCatchWindowUs) {
        return false;
    }

    int64_t edgeNs = atNs;
    if (self->m_capNowValid && self->m_capHz != 0) {
        /* Signed difference, so the counter may wrap. */
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
    /* Runs on the timing core: the GPIO driver binds its interrupt to the core
     * that enables the first pin. */
    gpio_config_t pps = {};
    pps.pin_bit_mask  = 1ull << kPpsPin;
    pps.mode          = GPIO_MODE_INPUT;
    pps.pull_up_en    = GPIO_PULLUP_DISABLE;
    /* Pulled down for an unconnected pin; the module drives it push-pull. */
    pps.pull_down_en = GPIO_PULLDOWN_ENABLE;
    pps.intr_type    = GPIO_INTR_POSEDGE;
    esp_err_t err    = gpio_config(&pps);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d: %s -- no PPS", static_cast<int>(kPpsPin), esp_err_to_name(err));
        return err;
    }

    err = kCaptureEdge ? beginCapture() : ESP_ERR_NOT_SUPPORTED;
    if (err != ESP_OK) {
        if (kCaptureEdge) {
            ESP_LOGW(TAG, "MCPWM capture unavailable (%s): falling back to an interrupt stamp",
                     esp_err_to_name(err));
        }
        /* Without the probe the edge still works, uncorrected. */
        const esp_err_t probe = pps_latency::begin(kPpsPin);
        if (probe != ESP_OK) {
            ESP_LOGW(TAG, "PPS latency probe unavailable: %s", esp_err_to_name(probe));
        }
        err = beginInterruptEdge();
    }
    return err;
}

/* An interrupt binds to the core that installs it. esp_ipc cannot do this (the
 * GPIO driver makes its own IPC call and would deadlock), so a short-lived task
 * pinned to the timing core does. */
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
        /* `c` is on the waiter's stack and may be gone after this store. */
        c->done.store(true, std::memory_order_release);
        xTaskNotifyGive(waiter);
        vTaskDelete(nullptr);
    };
    /* Room for the latency probe's timer and ETM setup. */
    if (xTaskCreatePinnedToCore(run, "pps_bind", 6144, &call, kTaskPriority, nullptr,
                                cores::kTiming) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    /* Ignore notifications from elsewhere. */
    while (!call.done.load(std::memory_order_acquire)) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    return call.err;
}

/* GPIO interrupt at level 3, above the UART and Ethernet handlers so it nests
 * over them, and in IRAM so it runs with the flash cache off. */
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
        ESP_LOGI(TAG, "PPS edge: GPIO interrupt on core %d (stamp on entry)",
                 static_cast<int>(xPortGetCoreID()));
    }
    return err;
}

/* MCPWM capture. CONFIG_MCPWM_ISR_CACHE_SAFE and CONFIG_MCPWM_CTRL_FUNC_IN_IRAM
 * (sdkconfig.defaults) keep it working with the flash cache off. */
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

    /* The capture timer cannot be read, so a PWM timer in the same group is the
     * "now" end, once checked to count at the same rate. */
    mcpwm_ll_timer_set_count_mode(MCPWM_LL_GET_HW(kCapGroup), kCapNowTimer,
                                  MCPWM_TIMER_COUNT_MODE_UP);
    mcpwm_ll_timer_set_start_stop_command(MCPWM_LL_GET_HW(kCapGroup), kCapNowTimer,
                                          MCPWM_TIMER_START_NO_STOP);

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

    /* Compare the two counters over 10 ms; on a mismatch only the interrupt
     * time is used. */
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

    /* Confirm the edge stamp's tick before the first one is taken. */
    mono::begin();

    /* Start at the rate the receiver was last heard on; it keeps its saved rate
     * across a power cut. */
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

    /* Bias drive is active low and starts off; the sense pin has a board
     * pull-up. */
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

    m_decoder.listen(this);
    m_begun       = true;
    m_baud        = kBauds[m_preferredIndex];
    m_nextScanUs  = esp_timer_get_time() + kFirstRateUs;
    m_nextCheckUs = esp_timer_get_time() + 1000000;

    /* If TIM-TP has not appeared 3 s in, its enable is resent on its own so the
     * ACK or NAK can be attributed to it. */
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

    /* Configure first: a receiver in factory NMEA mode fills the line. */
    configureModule();
    restoreTimeMode();

    /* Antenna bias only after configuration. */
    vTaskDelay(pdMS_TO_TICKS(1500));
    applyBias(true);
    ESP_LOGI(TAG, "antenna bias on");

    for (;;) {
        /* While u-center is attached the bridge reads and feeds the parser; the
         * rest of the loop carries on. */
        if (m_bridgeActive) {
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            const int got = uart_read_bytes(kUart, buf, sizeof(buf), pdMS_TO_TICKS(100));
            m_decoder.feed(std::span(buf, static_cast<std::size_t>(got > 0 ? got : 0)));
        }

        const int64_t nowUs = esp_timer_get_time();
        /* Published after a NAV-PVT, well clear of the next edge, or on a timer
         * when no frames arrive. */
        if (m_statusDue || nowUs - m_statusPublishedUs >= 1100000) {
            publishStatus();
        }
        /* TIM-TP is polled each second, in both class spellings, because the
         * configured rate is not honoured. In fixed mode this receiver was seen
         * to send no TIM-TP at all. */
        if (m_timTpCfg == 1 && nowUs - m_timTpPolledUs >= 1000000) {
            m_timTpPolledUs = nowUs;
            sendUbx(0x0D, 0x01, nullptr, 0);
            sendUbx(0x01, 0x27, nullptr, 0);
        }

        /* Silence re-arms the baud scan, except while a bridge client holds the
         * port. */
        if (!m_bridgeActive && m_frameSeen && m_lastFrameUs != 0 &&
            nowUs - m_lastFrameUs > kFrameSilenceUs) {
            m_frameSeen   = false;
            m_lastFrameUs = 0;
            ++m_rescans;
            /* Reset the decoder (a stuck frame is the likelier cause) and give
             * the current rate a full first-rate turn before moving. */
            m_decoder.reset();
            m_nextScanUs = nowUs + kFirstRateUs;
            ESP_LOGW(TAG, "no frames for %lld ms: rescanning the baud rate",
                     static_cast<long long>(kFrameSilenceUs / 1000));
        }
        /* Six poor seconds in a row count as silence, so a wrong rate that
         * parses the odd frame does not hold the scan. */
        if (m_secStartUs == 0) {
            m_secStartUs = nowUs;
        } else if (nowUs - m_secStartUs >= 1000000) {
            /* Counted during configuration too: a working rate answers it with
             * ACKs. */
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

            /* With no frames, sample the RX pin: no transitions is a wiring
             * fault, transitions without frames a rate or framing one. */
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
                /* Change the receiver first, keeping the old rate to restore if
                 * frames do not resume. */
                m_baudPrev = m_baud;
                /* This module took up to 8.7 s to answer at a new rate. */
                m_baudRevertAtUs    = esp_timer_get_time() + 4 * kFrameSilenceUs;
                m_baudRevertPending = true;
                uart_wait_tx_done(kUart, pdMS_TO_TICKS(50));
                /* This module does not ACK CFG-PRT even when it takes, so only a
                 * NAK stops the change. */
                const int8_t prt = sendCfgPrt(want);
                if (prt == 0) {
                    ESP_LOGW(TAG, "CFG-PRT rejected: rate unchanged");
                } else {
                    /* Save it, after a pause: saving before the port change lands
                     * stores the old rate. */
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
            /* Stay on the new rate until the revert deadline, or a first-rate
             * turn when only the host moved. */
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

        /* Frames at the new rate: the change took. Otherwise, at the deadline,
         * the old rate is written back once the scan finds the receiver. */
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
            /* Saved, since the failed rate was saved too. */
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
        /* Requests from HTTP, carried out here so one task owns the UART. */
        if (m_biasReq >= 0) {
            const bool on    = m_biasReq == 1;
            const bool wasOn = m_antBiasOn;
            m_biasReq        = -1;
            /* Supply control bit on or off; short/open detection and
             * power-down-on-short stay set either way. */
            const uint8_t cfg[4] = {static_cast<uint8_t>(on ? 0x0F : 0x0E), 0, 0, 0};
            const int8_t  br     = sendAndWait(0x06, 0x13, cfg, sizeof(cfg));
            /* The pin does the switching. */
            m_antLatched   = false;
            m_shortSamples = 0;
            applyBias(on);
            gpio_set_level(static_cast<gpio_num_t>(kAntDrivePin), on ? 0 : 1);
            ESP_LOGW(TAG, "antenna bias %s on request (module said %s)", on ? "ON" : "OFF",
                     br == 1   ? "accepted"
                     : br == 0 ? "refused"
                               : "nothing");
            sendUbx(0x0A, 0x09, nullptr, 0); /* re-read the supervisor at once */

            /* After bias off -> on the receiver did not recover its front end
             * (no satellites, pegged AGC) until reset; a hardware reset keeps
             * the almanac. */
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
            /* A mask change restarts acquisition, so reconfigure everything. */
            ESP_LOGW(TAG, "constellation mask 0x%02x requested", static_cast<unsigned>(m_gnssMask));
            configureModule();
        }
        if (m_svinCmd != 0) {
            const uint8_t cmd = m_svinCmd;
            m_svinCmd         = 0;
            /* CFG-TMODE2 (0x06 0x3D): this is an M8 at protocol 22, where TMODE3
             * is ACKed but does nothing. Status comes from TIM-SVIN. */
            uint8_t  tm[28]   = {};
            uint32_t fixedAcc = 0;
            uint32_t minDur   = 0;
            uint32_t accLimit = 0;
            /* Only a survey fills the mean. Without one, re-lock the held
             * position; never send (0,0,0), which put the clock 10 ms out. */
            const bool haveMean = !(m_svinMean[0] == 0 && m_svinMean[1] == 0 && m_svinMean[2] == 0);
            if (cmd == 3 && !haveMean && !m_fixedValid) {
                ESP_LOGW(TAG, "nothing to lock: no survey mean and no held position, "
                              "position left moving");
                m_svinCmd = 0;
            } else if (cmd == 1) {
                tm[0]  = 1; /* survey-in */
                minDur = m_svinDurS;
                /* mm. The receiver reports survey variance in mm^2, but both
                 * accuracy fields here are plain mm. */
                accLimit = m_svinAccMm;
            } else if (cmd == 3) {
                /* ECEF, as reported. The accuracy (sqrt of the variance, mm)
                 * affects the stated timepulse accuracy, so it must not be sent
                 * as a variance. */
                tm[0] = 2; /* fixed */

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
                /* Copied out of the volatile members for std::format. */
                if (cmd == 1) {
                    const auto hours = static_cast<uint32_t>(m_svinDurS / 3600);
                    const auto accMm = static_cast<uint32_t>(m_svinAccMm);
                    Settings::instance().setGpsSurvey(std::format("{},{}", hours, accMm));
                }
                if (cmd == 3) {

                    if (haveMean) {
                        portENTER_CRITICAL(&m_mux);
                        m_holdValidated = m_svinValid;
                        portEXIT_CRITICAL(&m_mux);
                    }
                    Settings::instance().setGpsFixedPos(std::format(
                        "{},{},{},{},{}", haveMean ? m_svinMean[0] : m_fixedMean[0],
                        haveMean ? m_svinMean[1] : m_fixedMean[1],
                        haveMean ? m_svinMean[2] : m_fixedMean[2],
                        /* mm, not the variance */
                        haveMean ? isqrt(m_svinAcc) : m_fixedAcc, m_holdValidated ? 1 : 0));
                }
                saveTimeMode();
                m_svinAtUs = 0;
                if (cmd == 1) {
                    ESP_LOGW(TAG, "survey-in started: %u s minimum, %u mm limit",
                             static_cast<unsigned>(minDur), static_cast<unsigned>(m_svinAccMm));
                    /* A limit below the current scatter will not be met. */
                    if (m_svinAcc != 0 && isqrt(m_svinAcc) > accLimit) {
                        ESP_LOGW(TAG,
                                 "  the limit is tighter than this site scatters (%u mm "
                                 "now): expect it to run the full duration",
                                 static_cast<unsigned>(isqrt(m_svinAcc)));
                    }
                } else if (cmd == 3) {
                    ESP_LOGW(TAG, "position locked fixed at %d %d %d mm", m_svinMean[0],
                             m_svinMean[1], m_svinMean[2]);
                    /* Also reported as hold_validated in /gps/info. */
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
        /* A delay change re-sends CFG-TP5 only, and clears the rate window: the
         * phase step would otherwise be fitted as a rate error and undone. */
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
        /* Retry NAKed CFG-ANT and TIM-TP enables up to three times, 10 s apart:
         * the boot burst can reach the receiver before it is ready. */
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
                /* Do not overwrite a bridge client's configuration; detaching
                 * re-arms this. */
                m_reconfigAtUs = 0;
            } else {
                const bool forced = m_reconfigForce;
                m_reconfigAtUs    = 0;
                m_reconfigForce   = false;
                m_skipAdopt       = forced;
                configureModule();
                m_skipAdopt = false;
                if (forced) {
                    /* A reset cleared the time mode; restore the hold without
                     * asking the still-booting module. */
                    reassertFix();
                }
            }
        }
        /* Re-assert the hold while settings still say "fixed" and no survey is
         * running. Wait for a fix and live edges first: TMODE2 restarts the
         * pulse generator. */
        if (m_holdAssertAtUs != 0 && m_tmode == 2 && !m_svinActive) {
            /* Already holding. */
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
            /* CFG-CFG, clearMask 0x1F, saveMask 0, loadMask 0. Per the M8
             * protocol spec, clearMask resets the saved (non-volatile)
             * configuration to defaults and loadMask is what would copy it to
             * RAM, so this leaves the running configuration as it is. */
            const uint8_t cfg[12] = {0x1F, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            ESP_LOGW(TAG, "receiver configuration cleared to defaults");
            sendUbx(0x06, 0x09, cfg, sizeof(cfg));
            /* Re-apply and re-save the working rate, which the clear removed from
             * the saved configuration. */
            requestBaud(m_baud, true);
            /* Forced, so the message set is re-sent even with a bridge client. */
            m_reconfigForce = true;
            m_reconfigAtUs  = esp_timer_get_time() + 3000000;
        }
        if (m_resetReq) {
            m_resetReq = false;
            /* CFG-RST. Hot keeps the almanac; cold clears it. */
            const bool cold  = m_resetCold;
            const bool hard  = m_resetHard;
            const bool power = m_resetPower;
            m_resetHard      = false;
            m_resetPower     = false;
            /* resetMode: 0x00 hardware reset, 0x02 software reset (GNSS only),
             * 0x04 hardware reset after shutdown. The hardware resets reload the
             * saved configuration and lose the message set, so it is re-sent. */
            const uint8_t mode   = power ? 0x04 : hard ? 0x00 : 0x02;
            const uint8_t rst[4] = {static_cast<uint8_t>(cold ? 0xFF : 0x00),
                                    static_cast<uint8_t>(cold ? 0xFF : 0x00), mode, 0x00};
            ESP_LOGW(TAG, "receiver %s%s start requested",
                     power  ? "POWER-CYCLE "
                     : hard ? "HARDWARE "
                            : "",
                     cold ? "cold" : "hot");
            if (hard || power) {
                /* Forced, as for the clear above. */
                m_reconfigForce = true;
                m_reconfigAtUs  = esp_timer_get_time() + 3000000;
            }
            sendUbx(0x06, 0x04, rst, sizeof(rst));
        }

        /* Short sense at 4 Hz: three shorted reads in a row cut the bias and
         * latch it off. */
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

        /* Poll MON-HW every 2 s. */
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

void GpsTime::dropPhcAnchor()
{
    m_edgeSeq.fetch_add(1, std::memory_order_release);
    portENTER_CRITICAL(&m_mux);
    m_anchorPhcNs = 0;
    m_ppsPtpNs    = 0;
    portEXIT_CRITICAL(&m_mux);
    m_edgeSeq.fetch_add(1, std::memory_order_release);
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
