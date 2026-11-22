#include "ntp/serve_path.h"

#include <algorithm>

#include "esp_timer.h"

ServePath &ServePath::instance()
{
    static ServePath path;
    return path;
}

int64_t ServePath::settle(int64_t nowUs, int64_t raiseToNs)
{
    /* Lock-free: the state is one word, updated by compare-and-swap. */
    const uint32_t nowMin = static_cast<uint32_t>(nowUs / kPeriodUs) & kMinuteMask;
    uint32_t       cur    = m_state.load(std::memory_order_relaxed);
    for (;;) {
        const uint32_t since = (nowMin - (cur & kMinuteMask)) & kMinuteMask;
        double         hold  = static_cast<double>(cur >> kMinuteBits);
        for (uint32_t i = 0; i < since && hold > static_cast<double>(kFloorNs); ++i) {
            hold *= 1.0 - kDecayPerMin;
        }
        const int64_t claim =
            std::clamp<int64_t>(std::max(static_cast<int64_t>(hold), raiseToNs), kFloorNs, kCapNs);
        const uint32_t next = (static_cast<uint32_t>(claim) << kMinuteBits) | nowMin;
        if (next == cur || m_state.compare_exchange_weak(cur, next, std::memory_order_relaxed)) {
            return claim;
        }
    }
}

void ServePath::observe(int64_t halfNs, uint32_t count)
{
    if (count < kMinSamples || halfNs < 0) {
        return;
    }
    const int64_t nowUs = esp_timer_get_time();
    if (nowUs < kSettleUs) {
        return;
    }

    settle(nowUs, halfNs);
    m_minutes.fetch_add(1, std::memory_order_relaxed);
}

int64_t ServePath::boundNs()
{
    return settle(esp_timer_get_time(), 0);
}
