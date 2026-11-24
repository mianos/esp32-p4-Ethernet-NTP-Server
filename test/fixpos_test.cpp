/* Host tests for the position hold's stored record: the text in `gpsFixedPos`
 * in, and the ECEF position plus accuracy the servo and CFG-TMODE2 use out.
 *
 * These compile main/gps/fixpos.h itself -- not a twin of it -- because the
 * cases here are the ones this box has actually produced: a legacy record that
 * stored the survey variance (mm^2) where the accuracy goes, a record with and
 * without the validated flag, and the (0,0,0) that an empty NVS slot decodes to.
 * A reader that mishandles the first put a 2.6 m position into the receiver as
 * 6.7 km; one that mishandles the last sends CFG-TMODE2 with no position, which
 * left the clock 10 ms out.
 *
 *     ./test/run.sh
 */
#include <cstdint>
#include <cstdio>
#include <string_view>

#include "gps/fixpos.h"

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

/* The record this box carries: a survey mean in ECEF centimetres, written by a
 * firmware that stored the variance, and marked validated. */
constexpr std::string_view kLegacy = "-464289925,255243016,-353910813,6762477,1";

void testLegacyVariance()
{
    fixpos::Hold h;
    EXPECT("legacy record parses", true, fixpos::parse(kLegacy, h));
    EXPECT("legacy: x", -464289925, h.x);
    EXPECT("legacy: y", 255243016, h.y);
    EXPECT("legacy: z", -353910813, h.z);
    /* 6762477 mm^2 is 2600 mm, not 6762477 mm. */
    EXPECT("legacy: variance becomes mm", 2600, h.accMm);
    EXPECT("legacy: validated kept", true, h.validated);

    /* The same record without the validated field is a position, unvalidated. */
    fixpos::Hold four;
    EXPECT("four fields parse", true, fixpos::parse(kLegacy.substr(0, kLegacy.rfind(',')), four));
    EXPECT("four fields: same accuracy", 2600, four.accMm);
    EXPECT("four fields: not validated", false, four.validated);
}

void testMillimetres()
{
    /* What this firmware writes: the accuracy already in mm, so it is not
     * square-rooted a second time. */
    fixpos::Hold h;
    EXPECT("mm record parses", true, fixpos::parse("-464289925,255243016,-353910813,2600,1", h));
    EXPECT("mm record: accuracy kept", 2600, h.accMm);
    EXPECT("mm record: validated", true, h.validated);

    EXPECT("not validated parses", true, fixpos::parse("1,2,3,100,0", h));
    EXPECT("not validated: flag clear", false, h.validated);
    /* Any non-zero fifth field reads as validated. */
    EXPECT("flag 5 parses", true, fixpos::parse("1,2,3,100,5", h));
    EXPECT("flag 5: validated", true, h.validated);

    /* Negative axes are half the globe's worth of positions. */
    EXPECT("negative axes parse", true, fixpos::parse("-1,-2,-3,100,0", h));
    EXPECT("negative axes: x", -1, h.x);
    EXPECT("negative axes: z", -3, h.z);
}

void testAccuracyBoundary()
{
    fixpos::Hold h;
    EXPECT("at the bound parses", true, fixpos::parse("1,2,3,60000,0", h));
    EXPECT("at the bound: kept", 60000, h.accMm);
    /* One past it is a variance: isqrt(60001) is 244. */
    EXPECT("past the bound parses", true, fixpos::parse("1,2,3,60001,0", h));
    EXPECT("past the bound: converted", 244, h.accMm);
    /* A zero accuracy is a position with nothing said about it. */
    EXPECT("zero accuracy parses", true, fixpos::parse("1,2,3,0,0", h));
    EXPECT("zero accuracy: kept", 0, h.accMm);
}

void testRefusal()
{
    fixpos::Hold sentinel;
    sentinel.x     = 7;
    sentinel.accMm = 7;

    /* An empty NVS slot and a cleared record both decode to (0,0,0), which is
     * the centre of the Earth and not a position: CFG-TMODE2 with it put the
     * clock 10 ms out. */
    EXPECT("zero position refused", false, fixpos::parse("0,0,0,100,1", sentinel));
    EXPECT("zero position four fields refused", false, fixpos::parse("0,0,0,100", sentinel));
    EXPECT("empty refused", false, fixpos::parse("", sentinel));
    EXPECT("refusal leaves x alone", 7, sentinel.x);
    EXPECT("refusal leaves accuracy alone", 7, sentinel.accMm);

    EXPECT("three fields refused", false, fixpos::parse("1,2,3", sentinel));
    EXPECT("two fields refused", false, fixpos::parse("1,2", sentinel));
    EXPECT("six fields refused", false, fixpos::parse("1,2,3,4,5,6", sentinel));
    EXPECT("trailing separator refused", false, fixpos::parse("1,2,3,4,", sentinel));
    EXPECT("word refused", false, fixpos::parse("north,west,up,1", sentinel));
    EXPECT("partial number refused", false, fixpos::parse("1x,2,3,4", sentinel));
    EXPECT("space refused", false, fixpos::parse(" 1,2,3,4", sentinel));
    /* Beyond what ECEF centimetres hold: 4000000000 cm is 40000 km out. */
    EXPECT("past 32 bits refused", false, fixpos::parse("4000000000,2,3,4", sentinel));
    EXPECT("past 32 bits negative refused", false, fixpos::parse("-4000000000,2,3,4", sentinel));
}

}  // namespace

int main()
{
    testLegacyVariance();
    testMillimetres();
    testAccuracyBoundary();
    testRefusal();

    if (gFailed == 0) {
        std::printf("fixpos: %d checks passed\n", gChecks);
        return 0;
    }
    std::printf("fixpos: %d of %d checks failed\n", gFailed, gChecks);
    return 1;
}
