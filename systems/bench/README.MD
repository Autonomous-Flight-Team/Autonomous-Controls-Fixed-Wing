# Bench Subsystem

## Overview

Moves the surfaces in BENCH without MAVLink.

- **`BenchSetpoints`**: surface setpoints, range-checked against each channel's min/max. One object in `main`, written by the USB console and by MAVLink `DO_SET_SERVO` (comms' `BenchServoSink`). `Actuators` reads them only in BENCH.
- **`bench_console`**: USB console commands, chained after `comms_console_line()`.

| Command | Does |
|---|---|
| `bench`, `bench exit` / `exit` | enter / leave BENCH (through StateBridge) |
| `arm`, `disarm`, `clear` | arm, disarm, fault clear (through StateBridge) |
| `us <ch> <µs>` | one surface setpoint; `<ch>` is a name or index |
| `center` | every surface back to trim |
| `sweep <ch>` / `sweep all` | one slow trim → max → min → trim cycle (4 s) |

## Rules worth knowing

- **BENCH keeps the motor locked.** The throttle channel is never accepted as a setpoint, and `Actuators` sends stop in BENCH whatever the setpoints say. To test the ESC, arm normally with the prop off. An ESC test mode would need an FSM change (drawio, `SystemState`, tests, comms XML).
- **Mode commands only go through StateBridge**: same guards and one-loop pulses as MAVLink. The console never sets FSM inputs itself.
- **Console arming needs no MAVLink signing** (it needs a physical cable). Replies are tagged `USB` so the source shows up in logs.
- `bench_console_line()` is a template on the bridge type, so it builds without comms on the branch. Any type with `arm(bool, bool)`, `bench(bool)` and `fault_clear()` returning `MAV_RESULT` works.
- Setpoints persist after leaving BENCH. Call `center_all()` on exit if they shouldn't come back next time.

## MAVLink DO_SET_SERVO

Comms (`command_handler.h`) defines the interface `BenchServoSink`. Comms rejects malformed values (servo not 1–16, not an integer, out of range) before calling it. `StateBridge` only passes values through while the system is in BENCH with no exit pending. Servo numbers are 1-based, as in MAVLink. Limits are checked here: `false` sends DENIED back to the ground.

Comms' interfaces are virtual, so the adapter lives in `src/` (main wiring), not in this pure library:

```cpp
class BenchSink : public BenchServoSink {
  public:
    explicit BenchSink(BenchSetpoints& sp) : m_sp(sp) {}
    bool set_bench_servo(uint8_t servo, uint16_t pwm_us) override {
        return m_sp.set_servo_1based(servo, pwm_us);
    }

  private:
    BenchSetpoints& m_sp;
};

static BenchSink bench_sink(bench);
bridge.set_bench_servo_sink(&bench_sink);
```

## Usage

```cpp
// setup()
static BenchSetpoints bench(vehicle.channels, vehicle.num_channels);

// console line handler
if (comms_console_line(line, reply, sizeof(reply))) { ... }
else if (bench_console_line(line, now_ms, bridge, bench, reply,
                            sizeof(reply))) { Serial.println(reply); }

// every loop
bench.update(now_ms);            // advances sweeps
actuator_inputs.bench = bench.frame();
```

## Tests

`test/test_bench`: setpoints, parser, and console → fake bridge → real system FSM → `Actuators`.
