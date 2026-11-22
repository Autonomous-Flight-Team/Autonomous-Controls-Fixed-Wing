#ifndef DEMAND_SELECT_H_
#define DEMAND_SELECT_H_

#include "system_state.h"
#include <cstdint>

// Which source drives the outputs. A pure function of the state machines:
// no mode enum of its own, so it can't disagree with them.

// Values are logged: never renumber, only add.
enum class OutputSource : uint8_t {
    kSafe = 0,    // safe_us on every channel, throttle stop
    kMirror = 1,  // copy the receiver (mux on the pilot): bumpless handback
    kBench = 2,   // bench setpoints on surfaces, throttle stop
    kControl = 3, // control/ output through the mixer and throttle gate
};

const char* to_string(OutputSource s);

// manual_active (any state)                -> kMirror
// IN_AIR_RESET (pilot is on the mux)        -> kMirror
// BENCH                                     -> kBench
// ACTIVE, DEGRADED, CRITICAL, EMERGENCY     -> kControl
// BOOT, PREFLIGHT, STANDBY, FAULT_GROUND    -> kSafe
OutputSource select_source(SystemState sys, bool manual_active);

#endif // DEMAND_SELECT_H_
