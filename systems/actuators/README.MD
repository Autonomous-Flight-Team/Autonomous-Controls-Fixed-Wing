# Actuators Subsystem

## Overview

Turns the state machines and inputs into a pulse width per servo/ESC channel. It has no armed state or mode of its own: everything follows from `SystemState`, `FlightState` and `manual_active` every loop.

- **`demand_select`**: which source drives the outputs (table below).
- **`Actuators`** (`actuators.h/.cpp`): the per-loop pipeline. Pure; `main` writes its `OutputFrame` with `pwm_out::write_us()`.
- **`mixer`**: airframe mixing as a weight table (conventional, V-tail and flying wing are just data).
- **`OutputChannel`**: normalized command → µs per channel (min/trim/max, reverse, rate limit, safe value). Successor of the old repo's `GenericServo`.
- **`throttle_gate`**: throttle authority from the FSMs' rules, the protocol-neutral `ThrottleCmd`, ESC config and PWM translation, and `EscArmTracker` (ESC has seen stop long enough, for PREFLIGHT).
- **`motor_monitor`**: throttle vs. battery current → raw `FaultId::kMotorEsc` bit only.
- **`pwm_out`**: the only hardware code. `pwm_out_teensy.cpp` (FlexPWM at 50 Hz, `GNC_HARDWARE_BUILD`) and `pwm_out_fake.cpp` (native, records writes), chosen at link time.

`control/` is reserved for the PID/control-law code. It produces a `SurfaceDemand` that `Actuators` mixes.

## What drives the outputs

| Condition | Surfaces | Throttle |
|---|---|---|
| `manual_active` (any state) | mirror the receiver's µs (bumpless handback) | mirror if armed, else stop |
| BOOT, PREFLIGHT, STANDBY, FAULT_GROUND | `safe_us` | stop |
| BENCH | bench setpoints (unset = trim) | stop: motor locked |
| IN_AIR_RESET | mirror the receiver | stop (disarmed) |
| ACTIVE, DEGRADED, CRITICAL, EMERGENCY | `control/` output through the mixer | `gate_throttle()` |

Until `control/` exists, `ActuatorInputs::control` is `nullptr` and the armed rows output safe values: flights stay on the servo mux (pilot).

## Rules worth knowing

- **Faults are raw bits only.** `motor_monitor` sets the condition; `FaultManager` debounces and escalates; the FSMs respond. Nothing here reacts to a fault directly: it only follows the state.
- **Throttle gate:** armed, and ON_GROUND only when the system is ACTIVE; LANDED and REJECTED_TAKEOFF cut the motor.
- **Bumpless handback:** while another source drives a channel, its rate limiter is synced to what was actually sent, so the first `control/` output after a handback is rate limited from there.
- **Mirroring is exact:** the receiver's µs go straight out (only `pwm_out`'s 900–2100 µs hard clamp applies), so both sides of the mux match.
- **Bad config drives nothing:** `actuator_config_valid()` needs exactly one throttle channel, valid channels, one mixer row per channel and an ESC stop value equal to the throttle's `safe_us` (midpoint for a bidirectional ESC).
- Enum values are logged: never renumber, only add.

## Usage

Once per loop, after the FSM updates (state README "Usage" steps 1–4):

```cpp
ActuatorInputs ai;
ai.system = sys.state();
ai.flight = fl.state();
ai.manual_active = fi.manual_active;
ai.rc = &rc;              // latest PilotInput from comms
ai.rc_fresh = rc_fresh;   // pilot_fresh()
ai.control = nullptr;     // control/ output once it exists
ai.bench = bench.frame(); // BenchSetpoints
actuators.update(ai, now_ms, &frame);
for (uint8_t i = 0; i < frame.num_channels; ++i)
    pwm_out::write_us(i, frame.us[i]);
```

At startup, call `pwm_out::begin(pins, n, safe)` with `actuators.safe_us(safe)` before anything else, so outputs never float. Feed `actuators.preflight_ok(now_ms)` into `SystemInputs::preflight_ok`.

## Telemetry to comms

Comms sends `SERVO_OUTPUT_RAW` and `RC_CHANNELS` (2 Hz each) from the sensors `TelemetrySource` passed to the `StateBridge` constructor, the same way it sends attitude and GPS. Comms stamps the time itself. Fill the samples in that source (`src/`, main wiring):

```cpp
bool servo_outputs(ServoOutputSample& s) override {
    s.count = output_telemetry_us(frame, s.us, 16); // what pwm_out was told
    return s.count > 0;
}

bool rc_inputs(RcInputSample& s) override {
    s.count = rc_telemetry_us(rc, rc_fresh, s.us, 18); // 0 when stale
    s.link_quality_pct = rc_link_quality_pct(rc_link_status()); // 255 = unknown
    return true; // a lost link still reports LQ 0 and no channels
}
```

## Tests

`test/test_actuators` (units) and `test/test_manual_controls` (integration: pilot → FaultManager → both FSMs → Actuators, wired as in the state README's Usage section).
