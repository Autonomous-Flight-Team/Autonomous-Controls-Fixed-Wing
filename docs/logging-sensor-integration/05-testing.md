# Part 5: Testing and verification

[← Overview](README.md)

All six native suites pass: 132 tests, 0 failures. The Teensy firmware was checked by compiling and linking it on a PC, because PlatformIO itself couldn't be installed where this work ran.

## The suites

| Suite | Tests | New? | What it proves |
| --- | --- | --- | --- |
| `test_state` | 44 | existing | System and flight state machines (untouched, still green) |
| `test_logging` | 36 | 3 tests replaced, 2 added | SD writes, ring buffer, drop/loss counters, and golden bytes for all 5 packet types |
| `test_fault_manager` | 18 | existing | Fault table and manager |
| `test_sensor_integration` | 12 | new | Dummy sensors through `SensorManager`, the logger and the SD stub, end to end |
| `test_ring` | 11 | existing | Ring buffer edge cases |
| `test_sensor_health` | 11 | new | Every NOMINAL / DEGRADED / FAILED transition, streak resets, saturation |

### What test_sensor_integration covers

It runs the real `SensorManager` loop for simulated time in 100 µs steps, then splits the bytes written to the fake SD card back into packets, checking every `FE ED ... DE AD` frame on the way:

- one simulated second logs exactly 100 IMU, 190 barometer, 10 GPS and 50 pitot packets, with 0 dropped
- every packet's payload length matches its sensor
- `begin()` logs the trust limits first
- a sensor failing every read is never logged, goes to FAILED, and leaves the others unaffected
- the same sensor recovers to NOMINAL and logs at full rate again once it stops failing
- a sensor that fails to start is FAILED but doesn't block the rest
- with logging off, sensors are still read and nothing is written
- `RateGate` fires on time, skips missed ticks instead of bursting, and survives `micros()` wrapping

## Verifying without PlatformIO

The network policy where this work ran blocked PlatformIO's package registry, so `pio test` and `pio run` were not available. Two stand-ins did the job instead.

**Native tests: g++ plus Unity, directly.** A script does exactly what `env:native` does. For each `test/test_*` folder, it compiles every `.cpp` under `systems/`, the stubs and that suite with `g++ -std=c++17 -Wall -Wextra` and Unity from GitHub, then runs the binary. Same sources, same include paths, same compiler family, same results.

**Teensy firmware: a PC compile and link with stand-in Arduino headers.** Small stand-ins for `Arduino.h`, `Wire.h`, `SPI.h`, `HardwareSerial` and `elapsedMillis` were written. Then the real libraries (LSM9DS1, u-blox, MS4525DO, the libdriver BMP390 C code), our drivers, logging, state and `main.cpp` were compiled with `GNC_HARDWARE_BUILD` and linked into one program. This catches:

- type errors and wrong library calls
- missing functions (the BMP390 glue)
- multiple definitions (the `Barometer.h` bug)

Both `teensy41` and `teensy41_dummy` link cleanly. What this **can't** catch: anything ARM-specific, timing, I2C traffic, and whether the wiring matches the pin map. That needs a real board ([Part 6](06-open-items.md)).

## The on-Teensy test suite that was removed

A `test_hw_sensors` suite was written to run **on** the Teensy over USB (`pio test -e teensy41`). It checked that each real sensor starts and reads plausible values at rest (about 1 g on the IMU, 40–120 kPa, pitot near 0 Pa), then logged to the SD card. It passed against the dummies on a PC, but it was deleted on review:

- it needs a tethered board with every sensor wired, so it would rarely run
- its pass limits were guesses that would need tuning on the bench
- it needed extra plumbing: a guard around `main.cpp`'s `setup()`/`loop()`, Unity output over Serial, and test filters

The `teensy41_dummy` build covers most of what that suite was for: flash it, watch the status line, decode the SD card.

## CI

`.github/workflows/ci.yml` now also runs `pio run -e teensy41_dummy`, so both firmware variants must compile on every PR. CI is where the real ARM Teensy build gets checked, so the first PR from this branch is the true check.
