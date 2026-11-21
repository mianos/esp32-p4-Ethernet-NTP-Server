#include "ntp/serve_path.h"

#include <algorithm>

#include "esp_timer.h"

ServePath &ServePath::instance()
{
    static ServePath path;
    return path;
}

void ServePath::decayTo(int64_t nowUs)
{
    if (m_lastUs == 0) {
        m_lastUs = nowUs;
        return;
    }
    const int64_t sinceUs = nowUs - m_lastUs;
    if (sinceUs < kPeriodUs) {
        return;
    }
    const int64_t minutes = sinceUs / kPeriodUs;
    double        hold    = static_cast<double>(m_holdNs);
    for (int64_t i = 0; i < minutes && hold > static_cast<double>(kFloorNs); ++i) {
        hold *= 1.0 - kDecayPerMin;
    }
    m_holdNs = std::max<int64_t>(static_cast<int64_t>(hold), kFloorNs);
    /* Keep the remainder, so a burst of observations does not stack up decay. */
    m_lastUs += minutes * kPeriodUs;
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

    portENTER_CRITICAL(&m_lock);
    decayTo(nowUs);
    m_holdNs = std::clamp<int64_t>(std::max(m_holdNs, halfNs), kFloorNs, kCapNs);
    portEXIT_CRITICAL(&m_lock);

    m_minutes.fetch_add(1, std::memory_order_relaxed);
}

int64_t ServePath::boundNs()
{
    portENTER_CRITICAL(&m_lock);
    decayTo(esp_timer_get_time());
    const int64_t hold = m_holdNs;
    portEXIT_CRITICAL(&m_lock);
    return hold;
}
