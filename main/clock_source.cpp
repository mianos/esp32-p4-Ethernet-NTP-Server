#include "clock_source.h"

#include <ctime>

#include "esp_log.h"
#include "esp_timer.h"

#include "ntp_time.h"
#include "settings.h"

namespace {

constexpr const char *TAG = "clock_source";

/* How long the receiver has to be quiet before the fallback engages.
 *
 * The lock itself is lost after three seconds without a pulse (ten minutes once
 * a rate is known), so this is on top of that: three refused edges are a pulse
 * fault that the receiver may well recover from on its own, and starting a
 * client plus a clock step for it would make every jam a handover. Fifteen
 * seconds is long enough to be sure and short enough that the clock is still
 * close to right when the upstream takes it. */
constexpr int64_t kDegradeAfterUs = 15000000;

/* How long the receiver has to hold before the clock comes back. The lock
 * machinery needs three regular edges and a measured rate of its own before it
 * calls itself locked; twenty seconds is that plus a margin, and it is the time
 * in which the servo re-measures the crystal instead of trusting whatever it
 * had before the outage. */
constexpr int64_t kRecoverAfterUs = 20000000;

/* The servo is held off for this long after the client's first sync, so the one
 * edge that lands on the same instant as the step cannot measure the step as an
 * error and undo it. The tick that flips the owner runs at 1 Hz. */
constexpr int64_t kSettleUs = 3000000;

/* How stale an upstream sync may be and still be served. Fifteen missed polls;
 * the same reasoning as the lock window it replaces: wide enough that one lost
 * reply does not flap the stratum, narrow enough that a WAN that quietly died
 * is caught inside a minute of drift at a cheap crystal's rate. */
constexpr time_t kUpstreamFreshS = 900;

/* A failed engage is retried on this interval rather than every second. The
 * failures here are structural (no server in the lease, no memory), so a
 * one-second retry would only fill the log. */
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
    m_sinceUs     = nowUs();
    m_quietSinceUs = m_sinceUs;
}

void ClockSource::applySettings()
{
    if (Settings::instance().timeSource() == Settings::TimeSource::Auto) {
        /* Starting the dwell over is right in both directions: switching *to*
         * auto should not fall back on the strength of a quiet minute that
         * happened while somebody was in gps mode. */
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
    /* Monotonic, not the wall clock: a step in the middle of an episode would
     * otherwise make the age jump or go backwards. */
    return static_cast<uint32_t>((nowUs() - m_sinceUs) / 1000000);
}

bool ClockSource::upstreamLocked() const
{
    const NtpTime &ntp  = NtpTime::instance();
    const time_t   last = ntp.lastSync();
    const time_t   now  = time(nullptr);
    /* Now *or* after: lastSync is whole seconds and is stamped at the sync, so
     * for the rest of that second now == last. Requiring now > last would have
     * the server answer stratum 16 for up to a second out of every sixty -- a
     * client that polled into it would drop a server that is in fact synced.
     * A clock stepped backwards after a sync still fails, which is right. */
    return last != 0 && now >= last && (now - last) < kUpstreamFreshS;
}

bool ClockSource::gpsSteers()
{
    switch (Settings::instance().timeSource()) {
    case Settings::TimeSource::Gps:
        return true;
    case Settings::TimeSource::Auto:
        break;
    default:
        /* off/dhcp/static: the client is the only owner, as it always was. */
        return false;
    }

    switch (owner()) {
    case ClockOwner::Gps:
        return true;
    case ClockOwner::Upstream:
        return false;
    case ClockOwner::Handing:
        break;
    }

    /* Handing: the receiver still owns the clock, right up to the moment the
     * client takes it. Once the client has a sync, wait out the settling window
     * and then leave the clock alone until the owner flips. */
    if (!upstreamLocked()) {
        m_settleUs = 0;
        return true;
    }
    const int64_t at = nowUs();
    if (m_settleUs == 0) {
        m_settleUs = at;
    }
    return at - m_settleUs < kSettleUs;
}

void ClockSource::observe(const bool pulseGood)
{
    if (Settings::instance().timeSource() != Settings::TimeSource::Auto) {
        return;
    }

    const int64_t at = nowUs();
    /* A holdover reads as unhealthy here even though the receiver still serves:
     * the pulse is what a returning receiver has to prove, and handing the clock
     * back mid-holdover would hand it back on a prediction. */
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

    const bool recovered =
        m_healthySinceUs != 0 && at - m_healthySinceUs >= kRecoverAfterUs;

    switch (owner()) {
    case ClockOwner::Gps:
        if (m_quietSinceUs != 0 && at - m_quietSinceUs >= kDegradeAfterUs) {
            setOwner(ClockOwner::Handing);
            engage();
        }
        break;

    case ClockOwner::Handing:
        if (recovered) {
            /* Stop the client first: between these two calls nobody owns the
             * clock, which is the safe way round. */
            disengage();
            setOwner(ClockOwner::Gps);
        } else if (upstreamLocked()) {
            /* On the sync, not on a timer: this is the instant the client's
             * step landed, so it is also the instant the receiver must stop. */
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
    m_sinceUs = nowUs();
    m_settleUs = 0;
}

void ClockSource::engage()
{
    /* Called on the transition into Handing, and again from observe() while a
     * failed engage is being retried. A client already up is only re-polled,
     * not rebuilt. */
    m_nextEngageUs        = nowUs() + kEngageRetryUs;
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
