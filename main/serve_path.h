/*
 * How far the served time actually moves, measured on the replies that went out.
 *
 * Dispersion has to answer "how wrong might this be" without hardware stamps.
 * The transmit probe cannot answer it: it measures one syscall from an idle
 * task, and the served error moved +6 to -66 us in nine minutes while the probe
 * median sat still at 297. What can answer it is the clients, because every
 * request carries the stamp they sent it at. Our receive stamp minus theirs is
 * their clock offset plus the path; across a minute the offset cancels and the
 * range that is left is drift, measured on real traffic instead of guessed.
 *
 * The range is taken per client and held as a maximum with a slow decay: an
 * excursion should raise the claim immediately and be forgotten over about an
 * hour. A floor keeps a quiet minute from advertising zero, and a cap keeps one
 * client stepping its own clock from claiming half a millisecond for the day.
 *
 * This is a bound, not a correction. Nothing steers on it, so a client with a
 * bad clock can only make the claim more generous, never the time wrong.
 */
#pragma once

#include <atomic>
#include <cstdint>

#include "freertos/FreeRTOS.h"

class ServePath {
public:
    static ServePath &instance();

    /* One minute of one client: half the range seen, and how many samples it
     * rests on. Fewer than kMinSamples is ignored, so one exchange cannot set
     * the claim. */
    void observe(int64_t halfNs, uint32_t count);

    /* The current claim in nanoseconds. Decays on read, so an idle server
     * forgets an excursion instead of advertising it forever. */
    int64_t boundNs();
    double  boundUs() { return static_cast<double>(boundNs()) / 1000.0; }

    /* Minutes folded in so far, so a reader can tell a measured claim from the
     * floor it starts at. */
    uint32_t minutes() const { return m_minutes.load(std::memory_order_relaxed); }

private:
    ServePath() = default;

    /* Never claim less than this: the capture interrupt, half a reply's own
     * time in the stack, and room for a quiet minute. */
    static constexpr int64_t kFloorNs = 40000;
    /* Where a boot starts, so the first minute is not optimistic before there is
     * anything measured. It decays to the floor if the path deserves it. */
    static constexpr int64_t kStartNs = 150000;
    /* A client that steps its clock must not set the claim for the day. */
    static constexpr int64_t kCapNs = 500000;
    /* Forgotten at this rate: 150 us gone in about an hour. */
    static constexpr int64_t kDecayNsPerMin = 2000;
    static constexpr int64_t kPeriodUs      = 60000000;
    static constexpr uint32_t kMinSamples   = 4;

    void decayTo(int64_t nowUs);

    portMUX_TYPE          m_lock = portMUX_INITIALIZER_UNLOCKED;
    int64_t               m_holdNs = kStartNs;
    int64_t               m_lastUs = 0;
    std::atomic<uint32_t> m_minutes{0};
};
