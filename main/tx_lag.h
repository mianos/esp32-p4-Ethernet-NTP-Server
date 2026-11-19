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
 * whole file becomes unnecessary), so this measures what it can: a datagram
 * sent to this device's own address and read straight back. The half round trip
 * is the stack's cost in one direction plus the readback, and it is a real
 * number rather than a named constant. It does not include the PHY's
 * serialisation, so it is an under-estimate at the wire -- which is the safe
 * direction for an offset, because an under-corrected transmit stamp biases a
 * client's offset by less.
 *
 * Probes run on their own low-priority task so a blocked probe never delays the
 * responder, and the result is clamped: a 5 ms sample from a busy moment must
 * not drag every reply with it.
 *
 * The receive side uses the same figure. Both stamps are software, both err by
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

    /* The correction in nanoseconds, 0 until there is a measurement. */
    int64_t lagNs() const { return m_lagNs.load(std::memory_order_relaxed); }

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

    /* Round trips behind the current figure: 0 means "not measured yet". */
    uint32_t samples() const { return m_samples.load(std::memory_order_relaxed); }

    /* Public so a server can hold one by value. */
    TxLag() = default;

private:

    static void taskEntry(void *arg);
    void        task();
    /* One probe, in nanoseconds; -1 when it did not come back. */
    int64_t     probe();

    std::atomic<int64_t>  m_lagNs{0};
    std::atomic<uint32_t> m_samples{0};
    TaskHandle_t          m_task = nullptr;
};
