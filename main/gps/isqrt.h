/* Integer square root, for the mm^2 variances a receiver reports turning into an
 * accuracy in mm.
 *
 * Its own header because the position record's parser (fixpos.h) needs it and
 * must stay free of ESP-IDF headers, so the parser can be exercised on a host
 * (test/fixpos_test.cpp). gps_internal.h includes this one, so there is a single
 * definition. */
#pragma once

#include <cstdint>

/* Floor of sqrt(v): 6762477 gives 2600. */
inline uint32_t isqrt(uint32_t v)
{
    uint32_t r   = 0;
    uint32_t bit = 1u << 30;
    while (bit > v) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (v >= r + bit) {
            v -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
        bit >>= 2;
    }
    return r;
}
