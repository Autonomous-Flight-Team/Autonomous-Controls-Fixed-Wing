# Part 6: Open items and lessons

[← Overview](README.md)

The code is ready for review but not proven on hardware. Before merging, run CI's real Teensy build and settle three decisions with the people who own the sensor code.

## Not yet verified

- [ ] **Real ARM build.** Open a PR so CI runs `pio run -e teensy41` and `-e teensy41_dummy` with the actual Teensy toolchain. A PC link catches most problems, but not all.
- [ ] **GPS baud and protocol.** `GOOSE_GPS_BAUD` defaults to 921600, the teammates' value. Out of the box the NEO-F10N talks 38400 baud NMEA, while the bolderflight library needs UBX NAV-PVT messages. Configure the receiver in u-center first, or the GPS will sit at FAILED.
- [ ] **Bench run with `teensy41_dummy`.** Flash it with an SD card in, let it run, press `q`, then decode the card with `decoder.py`. That proves the on-board logging path with no sensors attached.
- [ ] **Bench run with `teensy41`.** Wire the sensors per the pin map. All four should show OK in the 5 s status line, and the decoded log should show about 1 g on the IMU at rest, about 101 kPa, and a pitot reading near 0 Pa.

## Decisions for the team

| Question | Current choice | Who |
| --- | --- | --- |
| Health thresholds: `>=` (reaching the count triggers the change) or `>` (exceeding it)? | `>=` | Sensor owners |
| Brace style: the merge reformatted logging and sensor files to braces on their own line, against the GOOSE style (same-line braces) | Left as is, to avoid conflicts while people are pushing | Team, at a quiet moment |
| Should `sensor_trust_limits` become PascalCase like every other type? | Kept, with a lint suppression, so nothing else breaks | Sensor owners |

## Lessons

- **Never `git add` a cloned library.** Libraries go in `lib_deps`. A folder with its own `.git` is stored as an empty pointer, and everyone else gets nothing.
- **A comment that disagrees with the code is a bug.** The "sensors are excluded" comment above a commented-out exclusion cost real confusion.
- **Function bodies in headers need `inline`,** a template or a class. Header guards don't help across `.cpp` files.
- **Placeholders that return `false` fail silently.** `isFresh() { return false; }` compiles, runs, and means nothing is ever logged. Prefer a TODO that fails loudly.
- **Read the library, not just the compiler.** All nine driver bugs compiled cleanly. Only the library sources showed they were wrong.
- **Put the format in one place and test it from both ends.** `sensor_packets.h` plus golden bytes from `decoder.py`'s own format strings means the firmware and the decoder can't drift apart quietly.
- **Design for a laptop.** Hardware-free logic (`SensorHealth`, `SensorManager`, `RateGate`) plus dummies at the edges gave 23 new tests that run in under a second, with no board needed.
