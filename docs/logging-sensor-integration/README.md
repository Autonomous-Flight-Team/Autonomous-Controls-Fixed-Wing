# GOOSE: Untangling the logging_subsystem Branch

Nov 22, 2026 · Advik Sharma (with Claude)

## TL;DR

The `logging_subsystem` branch went from "doesn't build, tests don't compile, sensors would never have logged a byte" to a branch where all 132 native unit tests pass and the Teensy firmware compiles and links (checked on a PC) with both real and dummy sensors.

A merge of logging + sensors and a few quick fixes landed on the branch on Nov 22, 2026. They left it in a state where nobody could tell what was broken. This series walks through what was wrong, what changed and why, so anyone on the team can pick the code up without having to reverse-engineer it.

| | Before | After |
| --- | --- | --- |
| Native unit tests | 3 of 4 suites build; `test_logging` doesn't compile | 6 suites, 132 tests, 0 failures |
| Teensy firmware | Empty library folders, duplicate symbols, missing BMP390 glue | Compiles and links on a PC with real libraries + stand-in Arduino headers |
| Sensor data reaching the SD card | None: every `isFresh()` returned `false` | Every fresh sample logged at its rate, checked byte-for-byte |
| Sensor drivers | 9 logic bugs (wrong conversions, inverted checks, unconnected GPS) | Fixed, and health tracking tested |
| Swapping hardware for test data | Hard-coded values in `main.cpp` | `GOOSE_DUMMY_SENSORS` / `sensor_suite.h` picks real or dummy at compile time |
| Header guards | Mix of `#pragma once`, `IMU_`, `FAULT_TABLE_H_` | One scheme: `GOOSE_<FOLDER>_<FILE>_H_` |

## How it unfolded

It started with a confused `git reset --hard origin/...` and a "what is going on with logging_subsystem?" Git turned out to be fine. The trouble was in the commits themselves.

1. **Diagnose.** The local branch matched `origin/logging_subsystem` and no work was lost. On top of Advik's last logging commits sat five new ones from lightningstar425 and Garcia. They merged sensors into logging, and the first one's message admits "has library linking issues".
2. **Fix the build config.** Four libraries had been committed as empty submodule pointers, and `lib_dir` pointed at them. The "sensors aren't built yet" exclusion was commented out, but the comment above it still said they were excluded. *([Part 1](01-repo-and-build.md))*
3. **Get the tests compiling again.** The merge deleted the dummy loggers `test_logging` used and dropped a declaration. Sensor data structs and their byte encoding moved into one hardware-free header, `sensor_packets.h`, with golden-byte tests checked against `decoder.py`. *([Part 2](02-logging-and-sensors.md))*
4. **Find the multiple-definition bug.** `Barometer.h` defined two whole functions, so every `.cpp` that included it got its own copy. The first fix was `inline`, the cleaner fix moved the bodies into `Barometer.cpp`. *([Part 4](04-native-vs-hardware.md))*
5. **Full integration.** Real drivers got checked against their library sources, which turned up nine bugs. A `SensorManager` now polls and logs every sensor. A compile-time switch picks real or dummy sensors. The missing BMP390 I2C glue got written. *([Part 2](02-logging-and-sensors.md), [Part 3](03-driver-bugs.md))*
6. **A hardware test suite, then deleted.** An on-Teensy test suite was written. It was removed after review because a tethered unit test wasn't worth the upkeep. *([Part 5](05-testing.md))*
7. **Header guards and the Arduino split.** Every `#pragma once` became a `GOOSE_..._H_` guard. Hardware-only code now sits behind `GNC_HARDWARE_BUILD`, so the native build can't accidentally pull in the real Arduino headers. *([Part 4](04-native-vs-hardware.md))*

## The series

Each part stands alone. If you only read one, read Part 4: it explains the C++ build mechanics everything else depends on.

| Part | What it covers | Read it if you... |
| --- | --- | --- |
| [1. Repo & build](01-repo-and-build.md) | Git state, empty submodule pointers, `platformio.ini` environments | ever hit "library not found" or wonder what `lib/` is for |
| [2. Logging ↔ sensors](02-logging-and-sensors.md) | `sensor_packets.h`, the decoder contract, `SensorManager`, `RateGate`, the new `main.cpp` | add a sensor or touch the log format |
| [3. Driver bugs](03-driver-bugs.md) | Nine driver and health-tracking bugs, before and after | own a sensor driver |
| [4. Native vs hardware](04-native-vs-hardware.md) | Multiple definitions, header guards, `GNC_HARDWARE_BUILD`, real vs dummy sensors | write any C++ in this repo |
| [5. Testing](05-testing.md) | The six test suites, how the build was checked without PlatformIO, CI | write or run tests |
| [6. Open items](06-open-items.md) | What still needs a real board, decisions for the team, lessons | review or merge this |

The changes touch about 30 files under `systems/`, `test/`, `src/main.cpp`, `platformio.ini`, `.github/workflows/ci.yml` and `systems/README.MD`.
