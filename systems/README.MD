# Systems

Each GOOSE system lives in its own folder here, with every `.h` next to its `.cpp`.
`src/` only holds `main.cpp`; `platformio.ini` pulls these folders into the build
(`build_src_filter`) and adds each one to the include path, so code includes
headers by name (`#include "logging_subsystem.h"`), never by relative path.

| Folder     | What it is                                              | Firmware (`teensy41`) | Unit tests (`native`) |
|------------|---------------------------------------------------------|-----------------------|-----------------------|
| `logging/` | Binary log packets → ring buffer → SD card, plus `decoder.py` | yes            | `test_logging`, `test_ring` |
| `state/`   | System FSM, flight FSM, fault table, fault manager      | yes                   | `test_state`, `test_fault_manager` |
| `sensors/` | IMU, barometer, GPS, pitot drivers and sensor health    | **not yet** (see below) | no (needs hardware libs) |

## Adding a system

1. Make `systems/<name>/` and put the `.h` and `.cpp` files there together.
2. Add `-I systems/<name>` to `system_includes` in `platformio.ini`.
3. If it has no Arduino/hardware dependencies, also add it to the `native` env's
   `build_src_filter` and include paths so it can be unit tested.
4. Tests go in `test/test_<name>/` (one folder per suite, each with its own `main`).

## Known issues in `sensors/`

The sensor code has not been compiled yet, so it is excluded from the firmware build
(`-<../systems/sensors/>` in `platformio.ini`). Before re-enabling it:

- `Barometer.cpp` calls `sensor_init()` before it is declared (declare it in
  `Barometer.h` or move it above `begin()`), and `Barometer::begin()` has no `return`.
- `driver_bmp390_shot.h` and `driver_bmp390_interface.h` live in the libdriver bmp390
  repo's `example/` and `interface/` folders, which PlatformIO does not build; the
  `bmp390_interface_*` functions also need a Teensy (Wire) implementation.
