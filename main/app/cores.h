#pragma once

/* Core 1 runs only the PPS interrupt, so masked stretches from other work do
 * not delay it; tasks and their driver interrupts run on core 0 (an interrupt
 * belongs to the core that installs it). IDF still runs its tick, idle task
 * and flash-write IPC on core 1. */
namespace cores {
constexpr int kTiming = 1;
constexpr int kWork   = 0;
}  // namespace cores
