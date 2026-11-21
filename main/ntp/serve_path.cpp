#include "ntp/serve_path.h"

#include <algorithm>

#include "esp_timer.h"

ServePath &ServePath::instance()
{
    static ServePath path;
    return path;
}

void ServePath::decayed(int64_t nowUs, int64_t &holdNs, int64_t &lastUs)
{
    if (lastUs == 0) {
        lastUs = nowUs;
        return;
    }
    const int64_t sinceUs = nowUs - lastUs;
    if (sinceUs < kPeriodUs) {
        return;
    }
    const int64_t minutes = sinceUs / kPeriodUs;
    double        hold    = static_cast<double>(holdNs);
    for (int64_t i = 0; i < minutes && hold > static_cast<double>(kFloorNs); ++i) {
        hold *= 1.0 - kDecayPerMin;
    }
    holdNs = std::max<int64_t>(static_cast<int64_t>(hold), kFloorNs);
    /* Keep the remainder, so a burst of observations does not stack up decay. */
    lastUs += minutes * kPeriodUs;
}

int64_t ServePath::settle(int64_t nowUs, int64_t raiseToNs)
{
    /* The decay is a loop of double-precision multiplies, so it runs between
     * two short holds rather than inside one: the lock is taken with interrupts
     * off, and the reply path reads this for every packet. If another writer
     * got in between, the arithmetic is done again on what it left. */
    int64_t now = 0;
    for (int attempt = 0; attempt < 3; ++attempt) {
        portENTER_CRITICAL(&m_lock);
        const int64_t hadHold = m_holdNs;
        const int64_t hadLast = m_lastUs;
        portEXIT_CRITICAL(&m_lock);

        int64_t hold = hadHold;
        int64_t last = hadLast;
        decayed(nowUs, hold, last);
        hold = std::clamp<int64_t>(std::max(hold, raiseToNs), kFloorNs, kCapNs);

        portENTER_CRITICAL(&m_lock);
        const bool mine = m_holdNs == hadHold && m_lastUs == hadLast;
        if (mine) {
            m_holdNs = hold;
            m_lastUs = last;
        }
        now = m_holdNs;
        portEXIT_CRITICAL(&m_lock);
        if (mine) {
            break;
        }
    }
    return now;
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
