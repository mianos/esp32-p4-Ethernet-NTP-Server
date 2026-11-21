/*
 * How long the transmit path holds a reply before the wire.
 *
 * A software transmit timestamp is taken before sendto(), so it is early by
 * everything the stack, the driver and the MAC do afterwards. Left in, that
 * error halves into every client's measured offset -- the same shape as the
 * receive lag, with the opposite sign, which is why the two partly cancel and
 * why leaving one of them uncorrected is worse than it looks.
 *
 * There is no hardware transmit stamp without the 1588 unit (and with it, this
 * whole file becomes unnecessary), so this measures what it can: the duration
 * of a datagram handed to the stack for our own bound port. That is the
 * userspace half of the path, and a lower bound on the whole of it, since the
 * driver and the wire happen after the call returns. An under-correction is the
 * safe direction: it leaves a client's offset biased by less than nothing at
 * all.
 *
 * The figure is not a constant of the build. It is the duration of a syscall
 * through a live stack, so it moves with cache state, descriptor availability
 * and whatever else is runnable -- measured moving by 30 % across a night, in
 * both directions, in steps too fast for a thermal story. Two numbers come out
 * of the same window and they are not the same kind of number:
 *
 *   lagNs()      the centre, which is what a reply is corrected by;
 *   scatterNs()  the spread around it, which is what dispersion should be
 *                claiming. Reporting the centre as the dispersion says "we do
 *                not know this to better than its own size" while
 *                simultaneously correcting by it.
 *
 * The centre is a median over a short ring rather than an exponential average,
 * because the average had an effective memory of two minutes and a single bad
 * sample took that long to wash out: it was reporting the noise, not the path.
 *
 * Probes run on their own low-priority task so a blocked probe never delays the
 * responder, and the result is clamped: a 5 ms sample from a busy moment must
 * not drag every reply with it.
 *
 * The receive side uses the same centre. Both stamps are software, both err by
 * the same path, and the errors are opposite in sign, so one measurement
 * serves both and the offset a client computes is not left carrying the
 * difference between two guesses.
 */
#pragma once

#include <atomic>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class TxLag {
public:
    static TxLag &instance();

    /* Start the probe task. Idempotent; called with the responder. */
    esp_err_t begin();

    /* Compile-time switch, so an A/B run needs no settings plumbing. */
    static constexpr bool kEnabled = true;

    /* The correction in nanoseconds, 0 until there is a measurement. */
    int64_t lagNs() const { return enabled() ? m_lagNs.load(std::memory_order_relaxed) : 0; }

    /* The spread of the window the centre came from, in nanoseconds: the median
     * absolute deviation, so one stalled probe does not widen it. Zero until
     * there are enough samples to mean anything. */
    int64_t scatterNs() const { return m_scatterNs.load(std::memory_order_relaxed); }

    /* Runtime switch, so an A/B run can be done inside one boot instead of
     * across two reboots with the servo in different states. */
    void setEnabled(bool on) { m_enabled.store(on, std::memory_order_relaxed); }
    bool enabled() const { return m_enabled.load(std::memory_order_relaxed); }

    /* The same figure for the receive side.
     *
     * It is one number because it is one measurement of one path: a self-send
     * traverses the stack out and back, and the halves are the same stack doing
     * the same work. The two errors are opposite in sign, so what a client's
     * offset sees is their difference -- which cancels when they come from the
     * same probe, and does not when one of them is a constant. */
    int64_t rxLagNs() const { return lagNs(); }

    /* The same figure in microseconds, for reporting. */
    double lagUs() const { return static_cast<double>(lagNs()) / 1000.0; }
    double scatterUs() const { return static_cast<double>(scatterNs()) / 1000.0; }

    /* Round trips behind the current figure: 0 means "not measured yet". */
    uint32_t samples() const { return m_samples.load(std::memory_order_relaxed); }

    /* One real reply's sendto, in nanoseconds. The probe measures the path when
     * a probe task happens to run; a reply's own call is the path the client
     * got, and it happens while the responder is warm rather than from an idle
     * task, so the two distributions are not the same and the reply's is the
     * one that matters. Kept as a distribution, not an average, because it is
     * bimodal: about one probe in eight comes back 140 us faster, which is
     * either an idle path the replies never take or a state they sometimes do.
     * A single number cannot say which, so the run reports percentiles. */
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
