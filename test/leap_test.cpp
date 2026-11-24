/* Host tests for the leap path: NAV-TIMELS bytes in, and the leap indicator the
 * NTP reply and the PTP announce carry out.
 *
 * These compile main/gps/leap.h itself -- not a twin of it -- because the cases
 * below are the ones a receiver gets wrong: the two valid bits are independent,
 * the warning expires with the minute, and 2026 has no leap pending at all, so
 * the steady state is the one that runs for years while the interesting ones
 * never arrive. Only plain values are involved, so no ESP-IDF and no board.
 *
 *     ./test/run.sh
 */
#include <cstdint>
#include <cstdio>

#include "gps/leap.h"

namespace {

int gChecks = 0;
int gFailed = 0;

void expectEq(const char *what, long want, long got, int line)
{
    ++gChecks;
    if (want != got) {
        ++gFailed;
        std::printf("  FAIL %s:%d  %s: want %ld, got %ld\n", __FILE__, line, what, want, got);
    }
}

#define EXPECT(what, want, got) \
    expectEq((what), static_cast<long>(want), static_cast<long>(got), __LINE__)

/* The 24-byte payload, everywhere else zero. */
void putTimels(uint8_t *p, int8_t currLs, int8_t lsChange, int32_t timeToLs, uint8_t valid)
{
    for (std::size_t i = 0; i < leap::kTimelsLen; ++i) {
        p[i] = 0;
    }
    const uint32_t countdown = static_cast<uint32_t>(timeToLs);
    p[9]                     = static_cast<uint8_t>(currLs);
    p[11]                    = static_cast<uint8_t>(lsChange);
    p[12]                    = static_cast<uint8_t>(countdown);
    p[13]                    = static_cast<uint8_t>(countdown >> 8);
    p[14]                    = static_cast<uint8_t>(countdown >> 16);
    p[15]                    = static_cast<uint8_t>(countdown >> 24);
    p[23]                    = valid;
}

void testParse()
{
    uint8_t p[leap::kTimelsLen];
    leap::Announcement a;

    /* 2026's steady state: 18 s of GPS-UTC, nothing announced. */
    putTimels(p, 18, 0, 0, 0x03);
    EXPECT("steady state parses", true, leap::parseTimels(p, leap::kTimelsLen, a));
    EXPECT("steady state: offset known", true, a.offsetKnown);
    EXPECT("steady state: offset 18", 18, a.offsetS);
    EXPECT("steady state: event known", true, a.eventKnown);
    EXPECT("steady state: nothing pending", 0, a.changeS);

    /* A real announcement. */
    putTimels(p, 18, 1, 45, 0x03);
    EXPECT("+1 announced parses", true, leap::parseTimels(p, leap::kTimelsLen, a));
    EXPECT("+1 announced: change", 1, a.changeS);
    EXPECT("+1 announced: count-down", 45, a.inS);

    /* The valid bits are independent: a receiver can know the offset before it
     * knows the event, and a receiver with no time can know neither. */
    putTimels(p, 18, 1, 45, 0x01);
    EXPECT("offset-only parses", true, leap::parseTimels(p, leap::kTimelsLen, a));
    EXPECT("offset-only: offset kept", 18, a.offsetS);
    EXPECT("offset-only: no event", false, a.eventKnown);

    putTimels(p, 18, 1, 45, 0x02);
    EXPECT("event-only parses", true, leap::parseTimels(p, leap::kTimelsLen, a));
    EXPECT("event-only: no offset", false, a.offsetKnown);
    EXPECT("event-only: event kept", true, a.eventKnown);

    putTimels(p, 18, 1, 45, 0x00);
    EXPECT("valid 0 parses", true, leap::parseTimels(p, leap::kTimelsLen, a));
    EXPECT("valid 0: no offset", false, a.offsetKnown);
    EXPECT("valid 0: no event", false, a.eventKnown);

    /* Bit 2 is the event's date, not either of the two this reads. */
    putTimels(p, 18, 1, 45, 0x04);
    EXPECT("date-only parses", true, leap::parseTimels(p, leap::kTimelsLen, a));
    EXPECT("date-only: no offset", false, a.offsetKnown);
    EXPECT("date-only: no event", false, a.eventKnown);

    /* Signed fields: -1 arrives as 0xFF. */
    putTimels(p, 18, -1, 45, 0x03);
    leap::parseTimels(p, leap::kTimelsLen, a);
    EXPECT("change -1 sign-extends", -1, a.changeS);

    /* Past the event the count-down is negative. */
    putTimels(p, 18, 1, -5, 0x03);
    leap::parseTimels(p, leap::kTimelsLen, a);
    EXPECT("negative count-down sign-extends", -5, a.inS);

    /* A payload one byte short is refused, whatever a field would have read,
     * and the caller's values are left alone. */
    leap::Announcement sentinel;
    sentinel.offsetS = 7;
    sentinel.inS     = 7;
    putTimels(p, 18, 1, 45, 0x03);
    EXPECT("short payload refused", false, leap::parseTimels(p, leap::kTimelsLen - 1, sentinel));
    EXPECT("short payload: offset untouched", 7, sentinel.offsetS);
    EXPECT("short payload: count-down untouched", 7, sentinel.inS);
    EXPECT("null payload refused", false, leap::parseTimels(nullptr, leap::kTimelsLen, sentinel));

    /* Bytes past this version are not part of it: ignored, not misread. */
    uint8_t longer[leap::kTimelsLen + 2];
    putTimels(longer, 18, 1, 45, 0x03);
    EXPECT("longer payload parses", true, leap::parseTimels(longer, sizeof longer, a));
    EXPECT("longer payload: count-down", 45, a.inS);
}

/* The RFC 5905 window, one row per way it can be asked. */
struct Gate {
    const char *what;
    bool        offsetKnown;
    bool        eventKnown;
    int8_t      changeS;
    int32_t     inS;
    uint8_t     want;
};

constexpr Gate kGates[] = {
    {"+1 s, 45 s out", true, true, 1, 45, 1},
    {"+1 s, first second of the minute", true, true, 1, 1, 1},
    {"+1 s, last second of the minute", true, true, 1, 60, 1},
    {"+1 s, one second past the minute", true, true, 1, 61, 0},
    {"+1 s, an hour out", true, true, 1, 3600, 0},
    {"+1 s, at the event", true, true, 1, 0, 0},
    {"+1 s, after the event", true, true, 1, -1, 0},
    {"-1 s, 45 s out", true, true, -1, 45, 2},
    {"-1 s, last second of the minute", true, true, -1, 60, 2},
    {"-1 s, one second past the minute", true, true, -1, 61, 0},
    {"nothing announced", true, true, 0, 45, 0},
    {"offset unknown", false, true, 1, 45, 0},
    {"event time unknown", true, false, 1, 45, 0},
    {"+2 s is not a change", true, true, 2, 45, 0},
    {"-2 s is not a change", true, true, -2, 45, 0},
};

void testIndicator()
{
    for (const Gate &g : kGates) {
        leap::Announcement a;
        a.offsetKnown = g.offsetKnown;
        a.eventKnown  = g.eventKnown;
        a.changeS     = g.changeS;
        a.inS         = g.inS;
        EXPECT(g.what, g.want, leap::indicator(a));
    }
}

void testReply()
{
    /* Locked: the announcement reaches the wire unchanged. */
    EXPECT("locked, 61-second minute", 1, leap::forReply(true, 1, false));
    EXPECT("locked, 59-second minute", 2, leap::forReply(true, 2, false));
    EXPECT("locked, nothing announced", 0, leap::forReply(true, 0, false));

    /* Not locked is 3, whatever arrived before the lock went. */
    EXPECT("unlocked", 3, leap::forReply(false, 1, false));
    EXPECT("unlocked, nothing announced", 3, leap::forReply(false, 0, false));

    /* A kiss-o'-death answers 0: a client must not read the rate limit as a leap
     * warning, and RFC 5905 asks for no warning on a KoD. */
    EXPECT("kiss-o'-death", 0, leap::forReply(true, 1, true));
    EXPECT("kiss-o'-death, unlocked", 0, leap::forReply(false, 1, true));
}

void testTaiMinusUtc()
{
    /* TAI-UTC has been 37 s since 2017-01-01, so GPS-UTC 18 gives 37. */
    EXPECT("GPS-UTC 18", 37, leap::taiMinusUtc(18));
    EXPECT("GPS-UTC 17", 36, leap::taiMinusUtc(17));

    /* Nothing reported yet, and a negative that cannot be one: 18 s. */
    EXPECT("nothing reported", 37, leap::taiMinusUtc(0));
    EXPECT("negative", 37, leap::taiMinusUtc(-1));

    /* The field is an octet. */
    EXPECT("just under the clamp", 126, leap::taiMinusUtc(107));
    EXPECT("clamped", 127, leap::taiMinusUtc(108));
}

/* Octet 0 of an NTP reply as buildReply packs it, for a version 4 client: the
 * reply is mode 4 (server) and the request's version is 4. */
constexpr uint8_t octet0(uint8_t indicator)
{
    return static_cast<uint8_t>((indicator << 6) | (4 << 3) | 4);
}

void testChain()
{
    uint8_t p[leap::kTimelsLen];
    leap::Announcement a;

    /* A 61-second minute, 45 s out, on a locked server. */
    putTimels(p, 18, 1, 45, 0x03);
    EXPECT("chain: parses", true, leap::parseTimels(p, leap::kTimelsLen, a));
    const uint8_t up = leap::indicator(a);
    EXPECT("chain: warning is 1", 1, up);
    EXPECT("chain: reply carries it", 1, leap::forReply(true, up, false));
    EXPECT("chain: octet 0", 0x64, octet0(leap::forReply(true, up, false)));

    /* A 59-second minute. */
    putTimels(p, 18, -1, 45, 0x03);
    leap::parseTimels(p, leap::kTimelsLen, a);
    const uint8_t down = leap::indicator(a);
    EXPECT("chain: warning is 2", 2, down);
    EXPECT("chain: octet 0", 0xA4, octet0(leap::forReply(true, down, false)));

    /* The minute after the event: the receiver stops counting down and the
     * warning clears, which is what a client sees as the leap having happened. */
    putTimels(p, 18, 0, 0, 0x03);
    leap::parseTimels(p, leap::kTimelsLen, a);
    const uint8_t none = leap::indicator(a);
    EXPECT("chain: no warning after the event", 0, none);
    EXPECT("chain: octet 0", 0x24, octet0(leap::forReply(true, none, false)));

    /* The same announcement from a server that is not locked: the client is told
     * the clock is not synchronised instead, leap or no leap. */
    EXPECT("chain: unlocked drops the warning", 3, leap::forReply(false, up, false));
    EXPECT("chain: unlocked octet 0", 0xE4, octet0(leap::forReply(false, up, false)));

    /* And the offset that goes with it, for PTP. */
    EXPECT("chain: TAI-UTC", 37, leap::taiMinusUtc(a.offsetS));
}

}  // namespace

int main()
{
    testParse();
    testIndicator();
    testReply();
    testTaiMinusUtc();
    testChain();

    if (gFailed == 0) {
        std::printf("leap: %d checks passed\n", gChecks);
        return 0;
    }
    std::printf("leap: %d of %d checks failed\n", gFailed, gChecks);
    return 1;
}
