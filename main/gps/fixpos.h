/* The position hold as NVS keeps it: "x,y,z,acc[,validated]", ECEF in
 * centimetres and the accuracy in millimetres.
 *
 * Two things the stored text can mean that a reader has to get right. An older
 * firmware wrote the survey variance (mm^2) where the accuracy goes, so an
 * accuracy beyond 60 m is square-rooted on the way in: 6762477 is 2.6 m, not
 * 6.7 km, and no real accuracy is that large anyway. And (0,0,0) is not a
 * position -- it is what an empty or half-written record decodes to, and
 * CFG-TMODE2 with it left the clock 10 ms out.
 *
 * Plain values only, no ESP-IDF and no state, so test/fixpos_test.cpp can
 * exercise it on a host. The caller owns the storage. */
#pragma once

#include <algorithm>
#include <cstdint>
#include <string_view>

#include "app/text.h"
#include "gps/isqrt.h"

namespace fixpos {

/* Above this an "accuracy" is a variance in mm^2, not an accuracy: no held
 * position for timing is 60 m out and still worth keeping. */
constexpr uint32_t kMaxAccMm = 60000;

struct Hold {
    int32_t  x         = 0;
    int32_t  y         = 0;
    int32_t  z         = 0;
    uint32_t accMm     = 0;
    bool     validated = false;
};

/* False when the text is not a position, leaving `out` untouched. */
inline bool parse(std::string_view text, Hold &out)
{
    long long     x   = 0;
    long long     y   = 0;
    long long     z   = 0;
    unsigned long acc = 0;
    unsigned long val = 0;
    /* x,y,z,acc[,validated]; the four-field form is unvalidated. getList takes
     * exactly the fields it is given, so six are refused by both calls. */
    const bool five = text::getList(text, ',', x, y, z, acc, val);
    if (!five && !text::getList(text, ',', x, y, z, acc)) {
        return false;
    }
    if (x == 0 && y == 0 && z == 0) {
        return false;
    }
    /* ECEF centimetres: 32 bits holds every position on and near the globe. */
    if (x > INT32_MAX || x < INT32_MIN || y > INT32_MAX || y < INT32_MIN || z > INT32_MAX ||
        z < INT32_MIN) {
        return false;
    }
    const uint32_t stored = static_cast<uint32_t>(std::min(acc, 0xFFFFFFFFul));
    out.x                 = static_cast<int32_t>(x);
    out.y                 = static_cast<int32_t>(y);
    out.z                 = static_cast<int32_t>(z);
    /* Older records stored the variance (mm^2); convert, as adoptFixedPosition
     * has always done. */
    out.accMm     = stored > kMaxAccMm ? isqrt(stored) : stored;
    out.validated = five && val != 0;
    return true;
}

}  // namespace fixpos
