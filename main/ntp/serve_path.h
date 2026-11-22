/*
 * A dispersion term measured on real traffic. For each client, our receive
 * stamp minus its transmit stamp is its offset plus the path; over a minute
 * the offset is roughly constant, so half the range is path variation. The
 * claim rises at once, decays 4% a minute, and is held between a floor and a
 * cap. It only widens dispersion and never steers the clock.
 */
#pragma once

#include <atomic>
#include <cstdint>

#include "freertos/FreeRTOS.h"

class ServePath {
public:
    static ServePath &instance();

    /* One client's minute: half its range and the sample count (ignored below
     * kMinSamples). */
    void observe(int64_t halfNs, uint32_t count);

    /* The current claim in ns, decayed to now. */
    int64_t boundNs();
    double  boundUs() { return static_cast<double>(boundNs()) / 1000.0; }

    /* Minutes folded in so far. */
    uint32_t minutes() const { return m_minutes.load(std::memory_order_relaxed); }

    /* Kept well below kServeHonestNs (asserted in gps_status.cpp), so clients'
     * stamps alone cannot demote the server. */
    static constexpr int64_t kCapNs = 200000;

private:
    ServePath() = default;

    static constexpr int64_t kFloorNs = 40000;
    /* The claim at boot, before anything is measured. */
    static constexpr int64_t kStartNs = 150000;

    static constexpr double kDecayPerMin = 0.04;
    /* Observations in the first minutes after boot are ignored. */
    static constexpr int64_t  kSettleUs   = 180000000;
    static constexpr int64_t  kPeriodUs   = 60000000;
    static constexpr uint32_t kMinSamples = 4;

    int64_t settle(int64_t nowUs, int64_t raiseToNs);

    /* The claim (upper 18 bits) and the minute last decayed at (lower 14 bits,
     * compared modulo; wraps every 11 days). */
    static constexpr uint32_t kMinuteBits = 14;
    static constexpr uint32_t kMinuteMask = (1u << kMinuteBits) - 1;
    static_assert(kCapNs < (1 << (32 - kMinuteBits)), "the claim has to fit beside the minute");
    std::atomic<uint32_t> m_state{static_cast<uint32_t>(kStartNs) << kMinuteBits};
    std::atomic<uint32_t> m_minutes{0};
};
