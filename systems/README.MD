# Systems

Each GOOSE system lives in its own folder here, with every `.h` next to its `.cpp`.
`src/` only holds `main.cpp`; `platformio.ini` pulls these folders into the build
(`build_src_filter`) and adds each one to the include path, so code includes
headers by name (`#include "logging_subsystem.h"`), never by relative path.

| Folder     | What it is                                              | Firmware (`teensy41`) | Unit tests (`native`) |
|------------|---------------------------------------------------------|-----------------------|-----------------------|
| `logging/` | Binary log packets → ring buffer → SD card, plus `decoder.py` | yes            | `test_logging`, `test_ring` |
| `state/`   | System FSM, flight FSM, fault table, fault manager      | yes                   | `test_state`, `test_fault_manager` |
| `sensors/` | IMU, barometer, GPS, pitot drivers and sensor health    | yes                   | `test_sensor_health`, `test_sensor_integration` |

## Adding a system

1. Make `systems/<name>/` and put the `.h` and `.cpp` files there together.
2. Add `-I systems/<name>` to `system_includes` in `platformio.ini`.
3. If it has no Arduino/hardware dependencies, also add it to the `native` env's
   `build_src_filter` and include paths so it can be unit tested.
4. Tests go in `test/test_<name>/` (one folder per suite, each with its own `main`).

## Native vs hardware code

Every file in `systems/` is one of three kinds:

| Kind | Examples | Off-board (native tests) |
|------|----------|--------------------------|
| Pure C++, no Arduino at all | `state/*`, `ring_buffer.h`, `logging_interface.h`, `sensor_packets.h`, `SensorHealth.*`, `DummySensors.h`, `sensor_manager.h`, `rate_gate.h` | built and tested as-is |
| Uses Arduino/SdFat, but only a little | `logging_subsystem.cpp` (`millis()`, `Serial`, SD card) | built against the fakes in `test/stubs/` |
| Real hardware drivers | `ImuK`, `Barometer`, `Gps`, `Pito`, `bmp390_teensy_interface.cpp` | `.cpp` wrapped in `#ifdef GNC_HARDWARE_BUILD` (compiles to nothing); `.h` stops with an `#error` if included |

`GNC_HARDWARE_BUILD` is set only by the Teensy envs in `platformio.ini`.
Code that wants sensors includes `sensor_suite.h`, which hands out the real
drivers on the Teensy and `DummySensors.h` everywhere else, so it never
includes a driver header directly. `src/main.cpp` is Teensy-only.

Header guards are `GOOSE_<FOLDER>_<FILE>_H_` (e.g. `GOOSE_SENSORS_RATE_GATE_H_`).

## Sensors and logging

| File | What it is |
|------|------------|
| `sensor_packets.h` | Data structs + log payload encoding. Hardware-free. |
| `SensorHealth.*` | NOMINAL / DEGRADED / FAILED tracking from read results. Hardware-free. |
| `ImuK`, `Barometer`, `Gps`, `Pito` | Real drivers (LSM9DS1, BMP390, u-blox, MS4525DO). |
| `bmp390_teensy_interface.cpp` | Teensy I2C glue the libdriver BMP390 driver needs. |
| `DummySensors.h` | Same interface as the drivers, fake data, failure injection. |
| `sensor_manager.h` | Polls each sensor at its rate (`rate_gate.h`) and logs fresh samples. |
| `sensor_suite.h` | Picks real or dummy sensors at compile time (below). |

**Real vs dummy sensors**

| Build | Defines | Sensors |
|-------|---------|---------|
| `pio run -e teensy41` | `GNC_HARDWARE_BUILD` | real |
| `pio run -e teensy41_dummy` | `GNC_HARDWARE_BUILD`, `GOOSE_DUMMY_SENSORS` | dummy (bench-test logging with nothing attached) |
| `pio test -e native` | none | dummy |

**Tests**

All run natively (`pio test -e native`): `test_sensor_health` (state
machine), `test_sensor_integration` (dummy sensors through the manager,
logger and SD stub: rates, packet layout, failure and recovery), and the
golden-byte payload tests in `test_logging`.

If you change a struct or add a sensor, update `sensor_packets.h`,
`logging/decoder.py` (`DECODERS`), `DummySensors.h` and the golden-byte tests
together.

Hardware libraries come only from `lib_deps` in `platformio.ini`. Don't commit
library clones into `lib/`: git stores them as empty submodule pointers.
