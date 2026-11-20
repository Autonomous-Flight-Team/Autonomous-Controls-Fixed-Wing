# State Subsystem

## Overview

GOOSE runs two state machines side by side, fed by a fault manager and a fault table:

- **System FSM** (`system_state.h/.cpp`) — health and arming: BOOT → PREFLIGHT → STANDBY → ACTIVE, the fault states (DEGRADED, CRITICAL, EMERGENCY, FAULT_GROUND), BENCH and IN_AIR_RESET.
- **Flight FSM** (`flight_state.h/.cpp`) — what the plane is doing: takeoff, climb, cruise, loiter, RTL, landing, and the aborts and failsafes. It only advances when the system FSM allows it.
- **Fault table** (`fault_table.h/.cpp`) — one row per fault: class, escalation, recoverability and flight response.
- **Fault manager** (`fault_manager.h/.cpp`) — turns raw fault bits from the health monitors into what the FSMs read: debounced active faults, escalation, the highest class, recoverability and the flight response.

The design is `designs/GOOSE State Machines.drawio` (page 1 system, page 2 flight, page 3 faults). If the code and the diagram disagree, fix one of them.

## Technical Details

- Each FSM is a class with a `switch` on an `enum class`. State only changes through its private `transition()`, which runs exit/entry actions and records the entry time and the last transition for logging.
- Both FSMs and the fault manager are pure: they read only their inputs struct and `now_ms`. No sensors, hardware or timers inside, so they run unchanged on the PC.
- No heap, no virtual functions, no Arduino headers in this folder, so it builds on the `native` env for tests.
- Enum values and `FaultId` bit positions are written to logs and telemetry: never renumber, only add.
- `FaultId` value = bit index in `FaultMask` (one `uint32_t` holds every active fault).
- Elapsed times use unsigned subtraction (`now_ms - start`), so they stay correct when `millis()` wraps.
- `is_armed(SystemState)` is a free function shared by both FSMs. Inside `SystemStateFSM` call it as `::is_armed(...)`, because the member `is_armed()` hides it.

### System FSM rules worth knowing

- **Disarm is only honored when `landed`** (flight FSM in ON_GROUND, TAKEOFF_ROLL, REJECTED_TAKEOFF or LANDED). An airborne disarm is ignored. A disarm on the takeoff roll aborts the takeoff.
- **FAULT_GROUND latching:** after a flight that reached CRITICAL/EMERGENCY, or after a boot timeout, FAULT_GROUND needs `fault_clear_cmd`. A PREFLIGHT fault clears on its own.
- **CRITICAL recovery** goes to DEGRADED only if every critical fault in the episode was recoverable, the fault has stayed clear for `t_hold`, and CRITICAL entries this flight are below `n_crit_max`. Otherwise CRITICAL stays latched until landing.
- **IN_AIR_RESET** leaves only on `fault_clear_cmd`: after a reboot the flight FSM starts in ON_GROUND, so `landed` can't be trusted.

### Fault manager rules worth knowing

- **Debounce both ways:** a raw bit must hold for `debounce_ms` to raise a fault, and stay clear for `debounce_ms` to clear it. Exceptions: LOOP_OVERRUN and IN_AIR_RESET are raised on the first loop they're seen.
- **Escalation** (fault manager only, not the FSMs). Timers count from when the fault was raised, after debounce, and restart if it clears:
  - RC_LOST → CRITICAL after `t_rc`, only if `rc_required`.
  - GPS_LOST → EMERGENCY after `t_loiter`.
  - LOOP_OVERRUN → EMERGENCY after `n_overrun_max` misses in a row.
  An escalated fault stays escalated until it clears. A `static_assert` checks that `escalates_to` in the table matches these triggers.
- **Highest class wins** (`highest_class()`). Non-critical faults don't affect `recoverable()` or `critical_response()`.
- **`recoverable()`** is false if any active CRITICAL-or-worse fault is non-recoverable.
- **`critical_response()`** is the most drastic response among active CRITICAL-or-worse faults: EMERGENCY_LAND > RTL > FAILSAFE_LOITER. RTL beats FAILSAFE_LOITER so a plane with low battery and no GPS goes home once GPS returns (the flight FSM handles the no-GPS part through `gps_ok`).
- **IN_AIR_RESET is latched:** it stays active until `fault_clear_cmd`, then clears on the same loop the system FSM leaves IN_AIR_RESET, so PREFLIGHT doesn't bounce to FAULT_GROUND.
- **Events for logging:** `raised_mask()`, `cleared_mask()` and `escalated_mask()` hold what changed on this `update()`; `changed()` is true if any is non-zero.

### Flight FSM rules worth knowing

- Each `update()` runs in this order: MANUAL check → AIRBORNE overrides (system EMERGENCY/CRITICAL) → normal transitions. At most one transition per loop.
- **MANUAL** freezes the FSM. When the pilot hands back, it restarts from what the plane is doing: LOITER if airborne (FAILSAFE_LOITER without GPS), ON_GROUND if not. After a manual landing the FSM is still frozen in an airborne state, so disarm is ignored: hand back to the autopilot first (it goes to ON_GROUND), then disarm.
- **ROTATE is committed:** faults don't abort past V1; overrides apply once in CLIMB.
- **LANDING is exempt** from the EMERGENCY_LAND and FAILSAFE_LOITER overrides: finishing the landing beats turning away. No go-arounds during EMERGENCY.
- **No GPS beats the fault's response:** in CRITICAL, `!gps_ok` sends the plane to FAILSAFE_LOITER whatever `critical_response` says. FAILSAFE_LOITER leaves on `gps_ok` even if the GPS fault is still debouncing or in `t_hold`.
- **A latched CRITICAL resumes the mission once GPS is back** (design choice, Nov 2026): FAILSAFE_LOITER goes to CRUISE, or to RTL if RTL was already the intent. The system stays CRITICAL until landing, and the operator can still send `rtl_cmd`.
- **Go-arounds are counted per flight** (reset at TAKEOFF_ROLL), not per approach.
- **RTL is sticky:** it leaves only for DESCENT, or on `resume_cmd` once the system is out of CRITICAL.

## Usage

Once per control loop, in this order:

1. Health monitors set raw fault bits in `FaultManagerInputs::raw`; `FaultManager::update(fm_inputs, now_ms)`.
2. Copy `highest_class()` → `SystemInputs::highest_fault` and `recoverable()` → `SystemInputs::fault_recoverable`. Then `SystemStateFSM::update(system_inputs, now_ms)`.
3. Copy `critical_response()` → `FlightInputs::critical_response`, the system state from step 2 → `FlightInputs::system_state`, and set `gps_ok` straight from the GPS (not the debounced fault). Then `FlightStateFSM::update(flight_inputs, now_ms)`.
4. Feed `FlightStateFSM::is_landed()` back into next loop's `SystemInputs::landed`.
5. If either FSM reports `changed()`, log its `last_transition()`. If the fault manager reports `changed()`, log its event masks.
6. Clear all command fields.

**`fault_clear_cmd` goes to both** `FaultManagerInputs` and `SystemInputs` on the same loop.

**Commands are one-loop pulses.** The main loop edge-detects RC switches and GCS messages and sets a command for exactly one loop. A held level (e.g. an arm switch left on) would re-arm right after a disarm.

**Home capture:** arm is the STANDBY → ACTIVE/DEGRADED transition. Latch home position when `changed()`, `last_transition().from == SystemState::kStandby` and `is_armed()`. Checking `from` alone also fires on STANDBY → BENCH and STANDBY → PREFLIGHT.

## Tests

`test/test_state/test_state_machines.cpp` runs both FSMs together, wired as in Usage, at 50 Hz simulated time. It covers every state and transition in both FSMs, including `millis()` wrap.

`test/test_fault_manager/test_fault_manager.cpp` tests the fault manager alone (debounce, escalation, priorities, in-air reset), then with both FSMs wired as in Usage.

```bash
pio test -e native -f test_state
pio test -e native -f test_fault_manager
```

The native env must see this folder's sources: keep them under `lib/` (linked into tests automatically) rather than using `test_build_src = yes`, which would also pull in the Arduino `main.cpp`.

Add a test whenever you add a state, a transition or a fault response.
