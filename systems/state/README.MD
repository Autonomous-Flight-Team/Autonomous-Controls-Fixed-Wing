# State Subsystem

## Overview

GOOSE runs two state machines side by side, plus a fault table that feeds them:

- **System FSM** (`system_state.h/.cpp`) — health and arming: BOOT → PREFLIGHT → STANDBY → ACTIVE, the fault states (DEGRADED, CRITICAL, EMERGENCY, FAULT_GROUND), BENCH and IN_AIR_RESET.
- **Flight FSM** (`flight_state.h/.cpp`) — what the plane is doing: takeoff, climb, cruise, loiter, RTL, landing, and the aborts and failsafes. It only advances when the system FSM allows it.
- **Fault table** (`fault_table.h/.cpp`) — one row per fault: class, escalation, recoverability and flight response.

The design is `designs/GOOSE State Machines.drawio` (page 1 system, page 2 flight, page 3 faults). If the code and the diagram disagree, fix one of them.

## Technical Details

- Each FSM is a class with a `switch` on an `enum class`. State only changes through its private `transition()`, which runs exit/entry actions and records the entry time and the last transition for logging.
- Both FSMs are pure: they read only their inputs struct and `now_ms`. No sensors, hardware or timers inside, so they run unchanged on the PC.
- No heap, no virtual functions, no Arduino headers in this folder, so it builds on the `native` env for tests.
- Enum values and `FaultId` bit positions are written to logs and telemetry: never renumber, only add.
- `FaultId` value = bit index in `FaultMask` (one `uint32_t` holds every active fault).
- Time-based fault escalation (RC after `t_rc`, GPS after `t_loiter`, loop overrun count) belongs to the fault manager, not the FSMs.
- Elapsed times use unsigned subtraction (`now_ms - start`), so they stay correct when `millis()` wraps.
- `is_armed(SystemState)` is a free function shared by both FSMs. Inside `SystemStateFSM` call it as `::is_armed(...)`, because the member `is_armed()` hides it.

### System FSM rules worth knowing

- **Disarm is only honored when `landed`** (flight FSM in ON_GROUND, TAKEOFF_ROLL, REJECTED_TAKEOFF or LANDED). An airborne disarm is ignored. A disarm on the takeoff roll aborts the takeoff.
- **FAULT_GROUND latching:** after a flight that reached CRITICAL/EMERGENCY, or after a boot timeout, FAULT_GROUND needs `fault_clear_cmd`. A PREFLIGHT fault clears on its own.
- **CRITICAL recovery** goes to DEGRADED only if every critical fault in the episode was recoverable, the fault has stayed clear for `t_hold`, and CRITICAL entries this flight are below `n_crit_max`. Otherwise CRITICAL stays latched until landing.
- **IN_AIR_RESET** leaves only on `fault_clear_cmd`: after a reboot the flight FSM starts in ON_GROUND, so `landed` can't be trusted.

### Flight FSM rules worth knowing

- Each `update()` runs in this order: MANUAL check → AIRBORNE overrides (system EMERGENCY/CRITICAL) → normal transitions. At most one transition per loop.
- **MANUAL** freezes the FSM. When the pilot hands back, it restarts from what the plane is doing: LOITER if airborne (FAILSAFE_LOITER without GPS), ON_GROUND if not.
- **ROTATE is committed:** faults don't abort past V1; overrides apply once in CLIMB.
- **LANDING is exempt** from the EMERGENCY_LAND and FAILSAFE_LOITER overrides: finishing the landing beats turning away. No go-arounds during EMERGENCY.
- **RTL is sticky:** it leaves only for DESCENT, or on `resume_cmd` once the system is out of CRITICAL.

## Usage

Once per control loop, in this order:

1. Health monitors set raw fault bits; the fault manager debounces them and reports the highest active class.
2. `SystemStateFSM::update(system_inputs, now_ms)`
3. `FlightStateFSM::update(flight_inputs, now_ms)`, with `flight_inputs.system_state` from step 2.
4. Feed `FlightStateFSM::is_landed()` back into next loop's `SystemInputs::landed`.
5. If either FSM reports `changed()`, log its `last_transition()`.
6. Clear all command fields.

**Commands are one-loop pulses.** The main loop edge-detects RC switches and GCS messages and sets a command for exactly one loop. A held level (e.g. an arm switch left on) would re-arm right after a disarm.

**Home capture:** arm is the STANDBY → ACTIVE/DEGRADED transition. Latch home position when `changed()` and `last_transition().from == SystemState::kStandby`.

## Tests

`test/test_state/test_state_machines.cpp` runs both FSMs together, wired as in Usage, at 50 Hz simulated time. It covers bring-up, arming, CRITICAL recovery and latching, takeoff aborts, go-arounds, failsafes, emergency landing and MANUAL handback.

```
pio test -e native -f test_state
```

The native env must see this folder's sources: keep them under `lib/` (linked into tests automatically) rather than using `test_build_src = yes`, which would also pull in the Arduino `main.cpp`.

Add a test whenever you add a state, a transition or a fault response.