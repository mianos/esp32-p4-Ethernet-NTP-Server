#include "serve_path.h"

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
    m_holdNs              = std::max<int64_t>(m_holdNs - minutes * kDecayNsPerMin, kFloorNs);
    /* Keep the remainder, so a burst of observations does not stack up decay. */
    m_lastUs += minutes * kPeriodUs;
}

void ServePath::observe(int64_t halfNs, uint32_t count)
{
    if (count < kMinSamples || halfNs < 0) {
        return;
    }

    portENTER_CRITICAL(&m_lock);
    decayTo(esp_timer_get_time());
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
