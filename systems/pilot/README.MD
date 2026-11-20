# Pilot Subsystem

## Overview

The boundary between comms and manual controls.

- **`pilot_input.h`**: plain-data structs owned here and filled by comms (`rc_link`, not written yet): `PilotInput` (one calibrated frame plus the receiver's µs), `RcLinkStatus` (link facts only) and `PilotParams`.
- **`pilot.h/.cpp`**: pure helpers: CRSF tick conversion, the raw `kRcLost` bit, freshness, switches, `manual_active` and mirroring.

Manual controls never parses a link. Comms reports facts; the decisions are made here.

## Rules worth knowing

- **No failsafe flag:** an ELRS receiver just stops sending frames when it loses the transmitter. `rc_lost_raw()` is true with no frame yet, no frame for `rc_timeout_ms`, or LQ at or below `min_link_quality`. `FaultManager` does the debouncing (`debounce_ms`) and escalation (`t_rc`).
- **`manual_active`** comes from the CH7 bypass switch while the frame is fresh. A stale frame reads as not manual, because the ER8 failsafe puts CH7 on the Teensy side. Swap in a SEL sense input if one is added.
- **CRSF → µs:** `crsf_ticks_to_us()` uses the same integer math as ExpressLRS `CRSF_to_US()`, which the ER8 uses for its own PWM outputs (172 → 988, 992 → 1500, 1811 → 2012, rounded straight line). `rc_link` must use this function, so mirroring matches the receiver exactly. A 0.625 slope matches the three anchors but is 1 µs off at about a quarter of the ticks in between.

## Usage

Each loop, at the start (before the fault manager):

```cpp
rc_link_update(now_ms);                 // comms
rc_link_latest(&rc);
const bool fresh = pilot_fresh(rc, pilot_params, now_ms);
if (rc_lost_raw(rc_link_status(), pilot_params, now_ms))
    fm_inputs.raw |= fault_bit(FaultId::kRcLost);
// ... FaultManager, system FSM ...
flight_inputs.manual_active = manual_active(rc, fresh);
```

## Tests

`test/test_pilot`. `rc_link` doesn't exist yet: inputs are built by hand.
