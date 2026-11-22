/*
 * Software transmit-path delay, for replies sent through sendto() without a
 * hardware stamp: the transmit timestamp is written before the call, so it is
 * early by at least the call's duration. That duration is probed every 5 s on
 * a low-priority task with a datagram to our own address; the median of the
 * last 15 is the correction (lagNs) and their median absolute deviation the
 * spread (scatterNs). It is a lower bound: the driver and wire come after the
 * call returns.
 *
 * Replies sent as raw frames on the 1588 path do not use it.
 */
#pragma once

#include <atomic>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class TxLag {
public:
    static TxLag &instance();

    /* Start the probe task; idempotent. */
    esp_err_t begin();

    static constexpr bool kEnabled = true;

    /* The correction in ns; 0 until measured or when disabled. */
    int64_t lagNs() const { return enabled() ? m_lagNs.load(std::memory_order_relaxed) : 0; }

    /* Median absolute deviation in ns. */
    int64_t scatterNs() const { return m_scatterNs.load(std::memory_order_relaxed); }

    /* Runtime switch for A/B runs within one boot. */
    void setEnabled(bool on) { m_enabled.store(on, std::memory_order_relaxed); }
    bool enabled() const { return m_enabled.load(std::memory_order_relaxed); }

    /* Unused; the receive side is covered by ServePath. */
    int64_t rxLagNs() const { return lagNs(); }

    double lagUs() const { return static_cast<double>(lagNs()) / 1000.0; }
    double scatterUs() const { return static_cast<double>(scatterNs()) / 1000.0; }

    /* Probes so far. */
    uint32_t samples() const { return m_samples.load(std::memory_order_relaxed); }

    /* One real reply's send duration in ns, reported as percentiles each
     * minute to compare with the probe. */
    void recordReply(int64_t ns);

    /* Public so a server can hold one by value. */
    TxLag() = default;

private:
    static void taskEntry(void *arg);
    void        task();
    /* One probe, in nanoseconds; -1 when it did not come back. */
    int64_t probe();
    /* Recompute the centre and the spread from the ring. */
    void absorb(int64_t lag);

    std::atomic<int64_t>  m_lagNs{0};
    std::atomic<int64_t>  m_scatterNs{0};
    std::atomic<bool>     m_enabled{kEnabled};
    std::atomic<uint32_t> m_samples{0};
    TaskHandle_t          m_task = nullptr;

    /* Only the probe task touches the ring. */
    static constexpr uint32_t kRing         = 15;
    int64_t                   m_ring[kRing] = {};
    uint32_t                  m_ringPos     = 0;
    uint32_t                  m_ringFill    = 0;
    uint32_t                  m_tick        = 0;

    /* Replies, written by the responder and drained by the probe task. */
    static constexpr uint32_t kReplyRing          = 32;
    int64_t                   m_reply[kReplyRing] = {};
    std::atomic<uint32_t>     m_replyPos{0};
    std::atomic<uint32_t>     m_replyFlooded{0};
    void                      reportReplies();
};
