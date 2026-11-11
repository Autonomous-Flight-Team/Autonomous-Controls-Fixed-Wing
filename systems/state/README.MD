# State Subsystem

## Overview

GOOSE runs two state machines side by side, plus a fault table that feeds them:

- **System FSM** (`system_state.h/.cpp`) — health and arming: BOOT → PREFLIGHT → STANDBY → ACTIVE, the fault states (DEGRADED, CRITICAL, EMERGENCY, FAULT_GROUND), BENCH and IN_AIR_RESET.
- **Flight FSM** (`flight_state.h/.cpp`) — what the plane is doing: takeoff, climb, cruise, loiter, RTL, landing, and the aborts and failsafes. It only advances when the system FSM allows it.
- **Fault table** (`fault_table.h/.cpp`) — one row per fault: class, escalation, recoverability and flight response.

The design is `designs/GOOSE State Machines.drawio` (page 1 system, page 2 flight, page 3 faults). If the code and the diagram disagree, fix one of them.

## Technical Details

- Each FSM is a class with a `switch` on an `enum class`. State only changes through its private `transition()`, which records the entry time and the last transition for logging.
- No heap, no virtual functions, no Arduino headers in this folder, so it builds on the `native` env for tests.
- Enum values and `FaultId` bit positions are written to logs and telemetry: never renumber, only add.
- `FaultId` value = bit index in `FaultMask` (one `uint32_t` holds every active fault).
- Time-based fault escalation (RC after `t_rc`, GPS after `t_loiter`, loop overrun count) belongs to the fault manager, not the FSMs.

## Usage

Once per control loop, in this order:

1. Health monitors set raw fault bits; the fault manager debounces them and reports the highest active class.
2. `SystemStateFSM::update(system_inputs, now_ms)`
3. `FlightStateFSM::update(flight_inputs, now_ms)`, with `flight_inputs.system_state` from step 2.
4. Feed `FlightStateFSM::is_landed()` back into next loop's `SystemInputs::landed`.
5. If either FSM reports `changed()`, log its `last_transition()`.

Tests: `pio test -e native`.
