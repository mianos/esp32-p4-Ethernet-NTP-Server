/*
 * Which reference owns the clock, and which one the responder serves.
 *
 * Two sources exist -- the GPS with its pulse-per-second line, and an upstream
 * SNTP server -- and only one of them may write CLOCK_REALTIME at a time.
 * Whoever owns the clock is also what the responder answers for, because a
 * server corrected by one source while claiming another is describing a device
 * that does not exist. Owner and served reference are therefore one variable,
 * and this class is the only writer of it.
 *
 * The transition into a fallback is slow to fire and quick to be safe:
 *
 *   Gps       the receiver is locked, or holding over on its fitted rate
 *   Handing   the receiver has been quiet for fifteen seconds and the client
 *             is being started; the receiver still owns the clock until the
 *             client has actually synced
 *   Upstream  the upstream owns the clock, which happens on the client's first
 *             sync, not on a timer
 *
 * Handing exists to solve the ordering problem rather than to be a state
 * anybody wants: the receiver must keep the clock while the client is warming
 * up (it is the better clock and it is still right), but the instant the client
 * steps the clock the old owner has to stop, or its next edge measures the
 * step as an error and drags the clock straight back. So ownership changes on
 * the sync, and gpsSteers() also holds the servo off for a settling window
 * afterwards to cover the sub-second gap between the client's step and the
 * tick that notices it.
 *
 * Coming back is the same discipline reversed: the receiver has to hold for
 * twenty seconds, the client is stopped, and only then is the servo allowed to
 * steer -- stop-then-hand-back, never the other way around.
 *
 * All of this only runs under timeSource=auto. "gps" and "dhcp"/"static"
 * keep the single-source behaviour they were configured for.
 */
#pragma once

#include <atomic>
#include <cstdint>

/* Who owns CLOCK_REALTIME. */
enum class ClockOwner : uint8_t {
    Gps,      /* the receiver, including a holdover and a quiet not yet long
                 enough to hand over */
    Handing,  /* the receiver has gone quiet; the upstream is being polled */
    Upstream, /* the upstream owns the clock */
};

class ClockSource {
public:
    static ClockSource &instance();

    /* Seed the quiet timer. Call once at boot, after Settings and before the
     * receiver starts: a device that boots with no antenna falls back fifteen
     * seconds later instead of never. */
    void begin();

    /* The setting changed. Reconverge: anything other than "auto" forces the
     * owner back to the receiver and stops a running fallback. */
    void applySettings();

    /* Once a second from the GPS tick: is the pulse back and trustworthy? Not
     * the lock -- see the note in logLockChange(), the lock itself waits for
     * the servo that waits for this. Runs on the GPS task and nowhere else,
     * which is what lets the dwell timers be plain integers. */
    void observe(bool pulseGood);

    /* May the PPS servo write to CLOCK_REALTIME at this instant? Called from the
     * receiver's edge path, i.e. the GPS task. */
    bool gpsSteers();

    /* One atomic load, so the reply path can afford it. */
    ClockOwner owner() const { return m_owner.load(std::memory_order_relaxed); }

    /* The upstream is the reference and its last sync is recent enough to
     * serve. UpstreamTimeRef asks this on every reply, which is what turns a
     * fallback whose upstream has stopped answering back into stratum 16. */
    bool upstreamLocked() const;

    /* Seconds since the owner last changed: how long this episode has run. */
    uint32_t ageS() const;

    /* How many times this device has handed the clock over since boot. A
     * receiver that flickers produces episodes nobody sees unless they are
     * counted. */
    uint32_t engagements() const { return m_engagements; }

    static const char *name(ClockOwner owner);

private:
    ClockSource() = default;

    void setOwner(ClockOwner owner);
    void engage();
    void disengage();

    std::atomic<ClockOwner> m_owner{ClockOwner::Gps};

    /* Dwell timers, GPS task only, microseconds on the monotonic clock. One is
     * set when the receiver starts looking healthy and the other when it stops;
     * each is cleared by the opposite observation, so the pair cannot both be
     * running and neither can be left counting through a healthy minute.
     *
     * applySettings() also clears them, from the HTTP task, without a lock. A
     * tearing 64-bit read can only cost one edge its steering decision, and the
     * next second re-establishes either timer, so this stays plain integers. */
    int64_t m_quietSinceUs   = 0;
    int64_t m_healthySinceUs = 0;
    /* When the client first showed a lock during the current handover, 0 until
     * then. The settling window is measured from here. */
    int64_t m_settleUs = 0;

    /* When a failed engage may be retried. */
    int64_t m_nextEngageUs = 0;

    int64_t  m_sinceUs     = 0;
    uint32_t m_engagements = 0;
};
