/* The mono counter at the resolution a PPS edge needs.
 *
 * esp_timer reports whole microseconds. The counter underneath it is the
 * systimer, which the P4 runs at 16 MHz (XTAL through a fixed 2.5 divider), so
 * the same instant is available 62.5 ns at a time without leaving the time base
 * the servo already fits against. */
#pragma once

#include <cstdint>

#include "esp_attr.h"

namespace mono {

/* Nanoseconds on esp_timer's epoch. Falls back to esp_timer's microseconds
 * until begin() has confirmed the tick. Safe from an interrupt with the flash
 * cache off. */
int64_t nowNs();

/* Confirms the tick against esp_timer, so a wrong divisor cannot scale every
 * edge and reach the servo as a rate error no crystal has. False: nowNs() stays
 * on microseconds. */
bool begin();

}  // namespace mono
