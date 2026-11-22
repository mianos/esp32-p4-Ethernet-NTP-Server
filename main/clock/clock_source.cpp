#include "clock/clock_source.h"

#include <ctime>

#include "esp_log.h"
#include "esp_timer.h"

#include "clock/ntp_time.h"
#include "app/settings.h"

namespace {

constexpr const char *TAG = "clock_source";

/* Bad pulse for this long starts the fallback, so a brief fault does not. */
constexpr int64_t kDegradeAfterUs = 15000000;

/* Good pulse for this long hands the clock back. */
constexpr int64_t kRecoverAfterUs = 20000000;

/* Upstream sync age still served: 15 polls at 60 s. */
constexpr time_t kUpstreamFreshS = 900;

/* Retry interval for a failed engage (no server, no memory). */
constexpr int64_t kEngageRetryUs = 10000000;

int64_t nowUs()
{
    return esp_timer_get_time();
}

}  // namespace

ClockSource &ClockSource::instance()
{
    static ClockSource source;
    return source;
}

const char *ClockSource::name(ClockOwner owner)
{
    switch (owner) {
    case ClockOwner::Gps:      return "gps";
    case ClockOwner::Handing:  return "handing";
    case ClockOwner::Upstream: return "upstream";
    }
    return "gps";
}

void ClockSource::begin()
{
    m_sinceUs      = nowUs();
    m_quietSinceUs = m_sinceUs;
}

void ClockSource::applySettings()
{
    if (Settings::instance().timeSource() == Settings::TimeSource::Auto) {
        /* Restart the dwell, so time spent in another mode does not count. */
        m_quietSinceUs   = nowUs();
        m_healthySinceUs = 0;
        m_settleUs       = 0;
        return;
    }

    if (owner() != ClockOwner::Gps) {
        disengage();
        setOwner(ClockOwner::Gps);
    }
    m_quietSinceUs   = nowUs();
    m_healthySinceUs = 0;
    m_settleUs       = 0;
}

uint32_t ClockSource::ageS() const
{
    return static_cast<uint32_t>((nowUs() - m_sinceUs) / 1000000);
}

bool ClockSource::upstreamLocked() const
{
    const NtpTime &ntp  = NtpTime::instance();
    const time_t   last = ntp.lastSync();
    const time_t   now  = time(nullptr);
    /* now == last for the rest of the sync's second, so >= not >. */
    return last != 0 && now >= last && (now - last) < kUpstreamFreshS;
}

bool ClockSource::gpsSteers()
{
    switch (Settings::instance().timeSource()) {
    case Settings::TimeSource::Gps:  return true;
    case Settings::TimeSource::Auto: break;
    default:

        return false;
    }

    switch (owner()) {
    case ClockOwner::Gps:      return true;
    case ClockOwner::Upstream: return false;
    case ClockOwner::Handing:  break;
    }

    /* Handing: steer until the upstream has synced, then stop at once, or the
     * next edge would measure the client's step as an error and undo it. */
    if (!upstreamLocked()) {
        m_settleUs = 0;
        return true;
    }
    return false;
}

void ClockSource::observe(const bool pulseGood)
{
    if (Settings::instance().timeSource() != Settings::TimeSource::Auto) {
        return;
    }

    const int64_t at = nowUs();
    /* Holdover counts as a bad pulse: the clock is not handed back on a
     * prediction. */
    if (pulseGood) {
        if (m_healthySinceUs == 0) {
            m_healthySinceUs = at;
        }
        m_quietSinceUs = 0;
    } else {
        if (m_quietSinceUs == 0) {
            m_quietSinceUs = at;
        }
        m_healthySinceUs = 0;
    }

    const bool recovered = m_healthySinceUs != 0 && at - m_healthySinceUs >= kRecoverAfterUs;

    switch (owner()) {
    case ClockOwner::Gps:
        if (m_quietSinceUs != 0 && at - m_quietSinceUs >= kDegradeAfterUs) {
            setOwner(ClockOwner::Handing);
            engage();
        }
        break;

    case ClockOwner::Handing:
        if (recovered) {
            /* Stop the client before taking the clock back. */
            disengage();
            setOwner(ClockOwner::Gps);
        } else if (upstreamLocked()) {
            /* On the client's first sync, when its step has landed. */
            setOwner(ClockOwner::Upstream);
            ESP_LOGW(TAG, "fallback: the upstream owns the clock (receiver quiet %u s)",
                     static_cast<unsigned>((at - m_quietSinceUs) / 1000000));
        } else if (at >= m_nextEngageUs) {
            /* The first engage failed -- no server, or no memory. */
            engage();
        }
        break;

    case ClockOwner::Upstream:
        if (recovered) {
            disengage();
            setOwner(ClockOwner::Gps);
            ESP_LOGI(TAG, "receiver back: locked for %u s, the clock is the receiver's again",
                     static_cast<unsigned>((at - m_healthySinceUs) / 1000000));
        }
        break;
    }
}

void ClockSource::setOwner(const ClockOwner owner)
{
    if (m_owner.load(std::memory_order_relaxed) == owner) {
        return;
    }
    m_owner.store(owner, std::memory_order_relaxed);
    m_sinceUs  = nowUs();
    m_settleUs = 0;
}

void ClockSource::engage()
{
    /* A client already running is only re-polled. */
    m_nextEngageUs         = nowUs() + kEngageRetryUs;
    const bool      before = NtpTime::instance().running();
    const esp_err_t err    = NtpTime::instance().startFallback();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cannot start the fallback upstream client: %s (retrying in %d s)",
                 esp_err_to_name(err), static_cast<int>(kEngageRetryUs / 1000000));
        return;
    }
    if (!before) {
        ++m_engagements;
    }
}

void ClockSource::disengage()
{
    m_nextEngageUs = 0;
    NtpTime::instance().stopFallback();
}
