/*
 * Under timeSource=auto, which source owns CLOCK_REALTIME and is served: the
 * GPS or an upstream SNTP server. Other settings fix the owner.
 *
 *   Gps       the receiver is locked or holding over
 *   Handing   the pulse has been bad for 15 s and the SNTP client is starting;
 *             the receiver steers until the upstream has synced
 *   Upstream  from the client's first sync
 *
 * The receiver stops steering as soon as the upstream has synced, or its next
 * edge would undo the client's step. Back to Gps after 20 s of good pulse,
 * stopping the client first.
 */
#pragma once

#include <atomic>
#include <cstdint>

/* Who owns CLOCK_REALTIME. */
enum class ClockOwner : uint8_t {
    Gps,      /* the receiver, including holdover */
    Handing,  /* the receiver has gone quiet; the upstream is being polled */
    Upstream, /* the upstream owns the clock */
};

class ClockSource {
public:
    static ClockSource &instance();

    /* Start the quiet timer at boot, so a box with no antenna falls back. */
    void begin();

    /* After a settings change. Anything but "auto" stops a fallback. */
    void applySettings();

    /* Once a second from the GPS task: whether the pulse is good (not the
     * lock; see logLockChange()). */
    void observe(bool pulseGood);

    /* Whether the GPS servo may write CLOCK_REALTIME now. GPS task. */
    bool gpsSteers();

    ClockOwner owner() const { return m_owner.load(std::memory_order_relaxed); }

    /* The upstream's last sync is recent enough to serve. */
    bool upstreamLocked() const;

    /* Seconds since the owner last changed. */
    uint32_t ageS() const;

    /* Fallbacks started since boot. */
    uint32_t engagements() const { return m_engagements; }

    static const char *name(ClockOwner owner);

private:
    ClockSource() = default;

    void setOwner(ClockOwner owner);
    void engage();
    void disengage();

    std::atomic<ClockOwner> m_owner{ClockOwner::Gps};

    /* Dwell timers (esp_timer us), GPS task. applySettings() clears them from
     * the HTTP task without a lock; a torn read costs at most one decision. */
    int64_t m_quietSinceUs   = 0;
    int64_t m_healthySinceUs = 0;
    /* Written but no longer read: the settling window it timed was removed. */
    int64_t m_settleUs = 0;

    /* When a failed engage may be retried. */
    int64_t m_nextEngageUs = 0;

    int64_t  m_sinceUs     = 0;
    uint32_t m_engagements = 0;
};
