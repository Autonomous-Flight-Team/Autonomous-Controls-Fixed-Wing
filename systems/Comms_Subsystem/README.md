# Comms Subsystem

Radio link between GOOSE (Teensy 4.1) and the ground station: **MAVLink v2 with
message signing**, over an **RFD900x-US** running SiK peer-to-peer firmware. This page is
the interface document for the ground-station author. The one file to share is
[`mavlink/goose.xml`](mavlink/goose.xml) (it includes the standard `common.xml`).

Scope: heartbeats, mode commands, and telemetry and logs at rates you can change. There
is no parameter or mission protocol.

## Overview

| File | What it does |
|---|---|
| `mavlink/goose.xml` | GOOSE dialect: `GOOSE_STATE`, `GOOSE_LOG`, GOOSE commands, state/fault enums mirroring `systems/state/` |
| `mavlink/generated/` | C headers from `mavgen` (gitignored, never hand-edited; create with `uv run tools/gen_mavlink.py`, see [Generating the MAVLink headers](#generating-the-mavlink-headers)) |
| `comms_config.h` | Serial port, baud, ids, default rates, timeouts, feature flags |
| `comms.h/.cpp` | `comms_init()`, `comms_update()` (non-blocking), parse/dispatch, link state, logger hook |
| `stream_scheduler.h/.cpp` | Per-stream intervals, byte budget, slow-down from radio `txbuf` |
| `command_handler.h/.cpp` | Command interface comms calls, plus a stub for link-only bench tests |
| `state_bridge.h/.cpp` | Real implementation: FSM guards, one-loop command pulses, GOOSE_STATE from the FSMs, enum drift checks |
| `telemetry_source.h/.cpp` | Interface supplying state and sensor snapshots, plus a dummy source |
| `signing.h/.cpp` | Key and timestamp storage (EEPROM), unsigned allowlist, USB provisioning commands |
| `radio_config.h/.cpp` | Boot-time AT-command check of the radio, with optional AES |
| `byte_stream.h`, `comms_platform.h/.cpp` | Transport (Arduino `Stream` or in-memory pipe), clock and EEPROM (real or faked on native) |

Firmware wiring is in `src/main.cpp`: radio on `Serial1` at 57600, dummy telemetry, stub
command handler, USB console for keys.

## Identity

| | System id | Component id |
|---|---|---|
| Aircraft | `1` (`COMMS_SYSID`) | `1` (`MAV_COMP_ID_AUTOPILOT1`) |
| Ground station | any, 255 suggested | any, 190 (`MAV_COMP_ID_MISSIONPLANNER`) suggested |
| SiK radio (RADIO_STATUS) | `51` (ASCII `'3'`) | `68` (ASCII `'D'`) |

The aircraft accepts commands from any signed sender. A ground station is recognised by
a `HEARTBEAT` with `type = MAV_TYPE_GCS`.

## Radio settings (both radios must match)

The Teensy checks its own radio once at boot (`COMMS_RADIO_AUTOCONFIG`). It reads `ATI5`
and only writes values that differ, then sends `AT&W` and `ATZ`. Set the **ground radio
by hand** to the same values, for example with a serial terminal at 57600:

```
+++            (1 s of silence before and after; radio answers OK)
ATI5           (list current settings)
ATS3=117       NETID: must match COMMS_RADIO_NETID (factory default 25)
ATS2=64        AIR_SPEED kbps: must match COMMS_RADIO_AIR_SPEED
ATS6=1         MAVLINK framing on: the radio injects RADIO_STATUS
ATS15=0        encryption off (or 1 with AES, below)
AT&W           save
ATZ            reboot
```

Leave `S1` (serial speed 57) and `S5` (ECC 0) at their defaults. ECC doubles the air
usage.

**Optional radio AES-128** (`COMMS_RADIO_AES 1`). This runs inside the radio, so it
costs no Teensy CPU. It is separate from MAVLink signing: AES hides the traffic, signing
authenticates commands. Both radios need `ATS15=1` and the same key:

```
AT&E=<32 hex chars>   set the AES key        AT&E?   show it
```

On the Teensy, load the AES key over USB with `AESKEY <32 hex>`. It is applied at the
next boot. If AES is on but no key is stored, the radio is left unchanged and the boot
log says so. Change radio settings only on the ground: AT mode drops the link for about
3 s.

## Signing

All aircraft frames are signed. With `COMMS_REQUIRE_SIGNING 1` (the default), the
aircraft **drops every unsigned or badly signed frame**. That includes frames with a
wrong key, a replayed timestamp, or a timestamp more than 1 minute old. The one
exception is `RADIO_STATUS` from the radio (sysid 51, compid 68), which SiK always sends
unsigned. The library checks each signature against the 32-byte secret key, the
`link_id` and a 48-bit timestamp counted in 10 µs units since 2015-01-01 GMT.

**Provisioning the key.** The key lives only in Teensy EEPROM. It is never in git and
never sent over the radio. Connect USB (115200 baud) and type:

```
KEY <64 hex chars>   store the key (reply shows a fingerprint, never the key)
KEY?                 show fingerprint = first 6 bytes of SHA-256(key), as hex
KEY CLEAR            erase
STATS                link counters
```

To generate a key: `python3 -c "import secrets; print(secrets.token_hex(32))"`. To
check the ground side has the same key, compare fingerprints:
`hashlib.sha256(key).hexdigest()[:12]`.

**Ground side (pymavlink):**

```python
import os; os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil
conn = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600, source_system=255, source_component=190)
conn.setup_signing(key_bytes, sign_outgoing=True,
                   allow_unsigned_callback=lambda mav, msg_id: msg_id == 109)  # RADIO_STATUS only
```

`setup_signing` starts the ground timestamp from the PC clock. Keep that default and do
not pass `initial_timestamp`.

**Timestamps across reboots.** The aircraft saves its signing timestamp to EEPROM each
time it has advanced about 50 s, at every boot, and on disarm. At boot it starts 60 s
past the saved value, as ArduPilot does. It also adopts any newer timestamp it receives
from the ground. Because of that, its clock tracks the PC clock, and after a power cycle
the ground keeps accepting its frames. If the aircraft was off for longer than that
margin and the ground restarted too, the ground may reject aircraft frames as "old" for
a moment. This lasts until the first signed ground heartbeat arrives (≤ 0.5 s), after
which the aircraft adopts the ground's time.

## Uplink: what the ground must send

- **`HEARTBEAT` at 2 Hz**, signed, `type = MAV_TYPE_GCS`. `comms_gcs_link_ok()` is true
  while one arrived in the last `COMMS_GCS_TIMEOUT_MS` (3000 ms).
- **`COMMAND_LONG`**, signed, `target_system = 1`, `target_component = 1` (or 0). Use
  the standard retry scheme: resend with `confirmation` + 1 if no ACK arrives within
  about 1 s. A retry (`confirmation > 0`) of an identical command within 3 s is **not
  run again**. The aircraft repeats the original result instead. A new attempt with
  `confirmation = 0` is always run.
- `COMMAND_INT` is answered with `MAV_RESULT_COMMAND_LONG_ONLY`. Every other message is
  ignored.

### Commands

Comms authenticates and de-duplicates each command, then hands it to the
`CommandHandler`. With the real state machines that handler is `StateBridge`
(`state_bridge.h`). It checks the guards the FSM code applies, so the ACK tells the
truth, and on ACCEPTED it sets the matching FSM input for exactly one loop. The guards
below come from `systems/state/system_state.cpp` and `flight_state.cpp`.

| Command | Params | Guard (StateBridge, from the FSM code) | FSM input |
|---|---|---|---|
| `MAV_CMD_COMPONENT_ARM_DISARM` (400) | p1: 1 arm / 0 disarm (other values DENIED); p2 21196 (force) is ignored | arm: STANDBY, `preflight_ok`, no critical fault (already armed → ACCEPTED); disarm: armed and `is_landed()` (ON_GROUND, TAKEOFF_ROLL, REJECTED_TAKEOFF, LANDED). Never in the air, even forced | `arm_cmd` / `disarm_cmd` |
| `MAV_CMD_NAV_TAKEOFF` (22) | p1 pitch, p7 alt: ignored | system **ACTIVE** (not DEGRADED), flight ON_GROUND, `mission_valid`, not MANUAL | `takeoff_cmd` |
| `MAV_CMD_NAV_LOITER_UNLIM` (17) | params ignored: the FSM loiters at a point chosen by guidance | flight CRUISE (already LOITER → ACCEPTED), not MANUAL, no failsafe override | `loiter_cmd` |
| `MAV_CMD_NAV_RETURN_TO_LAUNCH` (20) | – | flight CLIMB, CRUISE or LOITER (already RTL → ACCEPTED), not MANUAL. During a failsafe only if it is an RTL failsafe | `rtl_cmd` |
| `MAV_CMD_GOOSE_RESUME_MISSION` (44103) | – | LOITER with a valid, unfinished mission; or RTL with a valid mission, system not CRITICAL, not yet over the landing site. Not MANUAL, no failsafe override | `resume_cmd` |
| `MAV_CMD_GOOSE_BENCH` (44102) | p1: 1 enter / 0 exit (other values DENIED) | enter: STANDBY, `preflight_ok`, no critical fault (already BENCH → ACCEPTED); exit: in BENCH (goes back through PREFLIGHT) | `bench_cmd` / `exit_bench_cmd` |
| `MAV_CMD_GOOSE_FAULT_CLEAR` (44101) | – | FAULT_GROUND with no critical fault active, or IN_AIR_RESET | `fault_clear_cmd` |
| `MAV_CMD_SET_MESSAGE_INTERVAL` (511) | p1 message id, p2 interval µs (-1 off, 0 default) | – | ACCEPTED; DENIED if the message is not a stream (including HEARTBEAT) |
| `MAV_CMD_GET_MESSAGE_INTERVAL` (510) | p1 message id | – | ACCEPTED, then `MESSAGE_INTERVAL` (-1 off, 0 not a stream) |
| `MAV_CMD_GOOSE_SET_LOG_RATE` (44100) | p1 logger system, p2 subsystem, p3 interval µs (-1 off, 0 default) | – | ACCEPTED; DENIED if p1/p2 not 0–255 or the log table (8 streams) is full |
| anything else | | | UNSUPPORTED |

Results:

- **ACCEPTED**: the input is set for the next loop.
- **DENIED**: a guard failed.
- **TEMPORARILY_REJECTED**: another command for the same FSM is already waiting for
  that loop. Only one system command and one flight command are passed on per loop.

The ARM/DISARM guard checks the system state machine; the others check the flight state
machine (BENCH and FAULT_CLEAR also check the system one).

"Failsafe override" means the aircraft is airborne and the system is EMERGENCY, or
CRITICAL with a fault response. In that case the flight FSM obeys the failsafe and
ignores commanded transitions. The guards use the FSM inputs from the last loop, so a
fault that appears in that one loop can still make the FSM ignore an ACCEPTED command.

**No LAND command.** The flight FSM has no land input, so `MAV_CMD_NAV_LAND` is answered
UNSUPPORTED. To land, send RTL. The FSM then flies RTL → DESCENT (over the landing site)
→ LANDING → LANDED.

Before the FSM was wired in, `StubCommandHandler` was used for link-only bench tests. It
moves a dummy state with simplified versions of the same guards.

An unsigned command gets **no ACK at all**: it is dropped before it is decoded.

## Downlink

Default rates are set in `comms_config.h` and can be changed at runtime. The byte column
is the worst-case signed frame size: payload + 12 header/CRC + 13 signature bytes.
MAVLink 2 trims trailing zero bytes, so most frames are shorter.

| Message | Default | Bytes | Notes |
|---|---|---|---|
| `HEARTBEAT` | 1 Hz, fixed | 34 | `type` FIXED_WING, `autopilot` GENERIC, `custom_mode` = `flight_state` + (`system_state` << 8); `base_mode` has SAFETY_ARMED in the armed set (ACTIVE, DEGRADED, CRITICAL, EMERGENCY — not IN_AIR_RESET) and MANUAL_INPUT_ENABLED while MANUAL is active |
| `GOOSE_STATE` | 2 Hz + on every change | 45 | see below |
| `SYS_STATUS` | 1 Hz | 68 | sensor present/health bits from `sensor_status`; battery; `errors_comm` = bad CRC + bad signature |
| `ATTITUDE` | 10 Hz | 53 | |
| `GLOBAL_POSITION_INT` | 4 Hz | 53 | |
| `GPS_RAW_INT` | 2 Hz | 77 | |
| `VFR_HUD` | 4 Hz | 45 | |
| `SCALED_IMU` | off | 49 | |
| `SCALED_PRESSURE` | 1 Hz | 41 | |
| `BATTERY_STATUS` | 1 Hz | 79 | pack voltage in `voltages[0]` |
| `GOOSE_LOG` | 1 Hz per logger stream | ≤ 97 | INFO records; WARN/ERROR records are sent at once |
| `STATUSTEXT` | events | ≤ 79 | WARN/ERROR log records, "no signing key" at boot |
| `COMMAND_ACK`, `MESSAGE_INTERVAL` | replies | 35, 31 | |

At the defaults the worst case is about 1.5 kB/s. That figure comes from the frame sizes
above and has not been measured on a link. The comms code caps itself at
`COMMS_TX_BYTES_PER_SEC` (3000 B/s); 57600 baud carries about 5.7 kB/s.

`HEARTBEAT.system_status`:

| GOOSE system state | `MAV_STATE` |
|---|---|
| BOOT | BOOT |
| PREFLIGHT | CALIBRATING |
| STANDBY, FAULT_GROUND, BENCH | STANDBY |
| ACTIVE, DEGRADED | ACTIVE |
| CRITICAL, IN_AIR_RESET | CRITICAL |
| EMERGENCY | EMERGENCY |

### GOOSE dialect (ids: messages 23100–23199, commands 44100–44199)

`GOOSE_STATE` (23100): `time_boot_ms`, `system_state`, `flight_state`, `active_faults`,
`critical_entries` (CRITICAL entries this flight, reset on disarm), `flags`, and
`sensor_status[8]`. Each `sensor_status` entry is a `GOOSE_SENSOR_STATUS`, the same
values as `sensors::sensor_status`, indexed by `GOOSE_SENSOR_INDEX`: IMU, BARO, GPS,
MAG, AIRSPEED, BATTERY, RC, spare.

The state and fault values are **the same numbers as the state machine code**
(`systems/state/`), which the owner never renumbers. `state_bridge.cpp` checks each
value with `static_assert`, so the build fails if the two drift apart.

`system_state` (`GOOSE_SYSTEM_STATE` = `SystemState`):

| Value | State | Armed |
|---|---|---|
| 0 | BOOT | |
| 1 | PREFLIGHT | |
| 2 | STANDBY | |
| 3 | FAULT_GROUND | |
| 4 | BENCH | |
| 10 | ACTIVE | yes |
| 11 | DEGRADED | yes |
| 12 | CRITICAL | yes |
| 13 | EMERGENCY | yes |
| 20 | IN_AIR_RESET | no (rebooted in the air, pilot in MANUAL) |

`flight_state` (`GOOSE_FLIGHT_STATE` = `FlightState`):

| Value | State |
|---|---|
| 0 | ON_GROUND |
| 1 | TAKEOFF_ROLL |
| 2 | ROTATE |
| 3 | LANDED |
| 10 | CLIMB |
| 11 | CRUISE |
| 12 | LOITER |
| 13 | RTL |
| 14 | DESCENT |
| 15 | LANDING |
| 20 | REJECTED_TAKEOFF |
| 21 | FAILSAFE_LOITER |
| 22 | EMERGENCY_LAND |

MANUAL is not a flight state. It is the hardware servo-mux bypass, reported in `flags`.

`active_faults` (`GOOSE_FAULT`, bit N = `FaultId` N = `FaultMask` bit N):

| Bit | Fault | Class |
|---|---|---|
| 0 | TELEMETRY_LOST | non-critical |
| 1 | REDUNDANT_SENSOR | non-critical |
| 2 | BARO | non-critical |
| 3 | MAG | non-critical |
| 4 | LOGGING_FAILURE | non-critical |
| 5 | RC_LOST | non-critical, escalates to critical |
| 6 | LOOP_OVERRUN | non-critical, escalates to emergency |
| 7 | GPS_LOST | critical |
| 8 | AIRSPEED | critical |
| 9 | GEOFENCE | critical |
| 10 | BATTERY_LOW | critical |
| 11 | IN_AIR_RESET | critical |
| 12 | BATTERY_CRITICAL | emergency |
| 13 | IMU_LOST | emergency |
| 14 | ESTIMATOR_DIVERGED | emergency |
| 15 | MOTOR_ESC | emergency |
| 16 | SERVO | emergency |

`flags` (`GOOSE_STATE_FLAGS`):

- bit 0 `ARMED`: system state is in the armed set.
- bit 1 `MANUAL_ACTIVE`: the pilot has the servos, and the flight FSM is frozen.

`GOOSE_LOG` (23101): `time_boot_ms`, `system`, `subsystem`, `level`, `len`,
`payload[64]`. This is one logger record, unchanged, so the ground decodes it with the
`DECODERS` table from the logging branch's `decoder.py`. Key the table by
`(system, subsystem)`, take `payload[:len]`, and read levels as 0x00 INFO, 0x0F WARN,
0xFF ERROR. The logger IDs are the same as the logging subsystem's:
`SENSOR 0x00`, `SUB_IMU 0x01`, `SUB_BARO 0x02`.

pymavlink cannot load `goose.xml` by name. Generate a module once (keep `common.xml`,
`standard.xml` and `minimal.xml` next to `goose.xml`):

```
uv run --no-project --with pymavlink==2.4.50 mavgen.py --lang=Python3 --wire-protocol=2.0 --output=goose.py goose.xml
```

Then set `mavutil.mavlink = goose` before opening the connection.
`tools/comms_bench.py` (`load_goose_dialect()`) shows the whole sequence.

### Rate control and back-pressure

- Each stream is a table row holding an interval in µs, where 0 means off. Rows are
  keyed by message id, or for log streams by logger `(system, subsystem)`. The minimum
  interval is 20 ms (50 Hz). `SET_MESSAGE_INTERVAL(GOOSE_LOG)` sets every log stream at
  once. `MAV_CMD_GOOSE_SET_LOG_RATE` sets one.
- Log streams are decimated, and the latest sample wins.
- Send priority: command ACKs, then heartbeat, then `GOOSE_STATE`, then events
  (STATUSTEXT, WARN/ERROR logs, MESSAGE_INTERVAL), then rate streams, most overdue
  first.
- `comms_update()` never waits. It writes a frame only when `availableForWrite()` has
  room for the frame's worst-case size and the byte budget allows it, so frames are
  never cut in half. When it cannot write, ACKs and events wait in small queues (4 and
  8 entries; the oldest is dropped when full). Rate streams skip that period and count
  a drop (`comms_stream_dropped()`). No backlog builds up.
- The radio reports `RADIO_STATUS.txbuf`, the percentage of its buffer that is free.
  Below 50 % the aircraft halves all stream rates, and below 25 % it quarters them. It
  returns to full rate after 5 s without a report.
- Each call reads at most 512 bytes and sends at most 8 frames.

## Logger hook

```cpp
comms_forward_log(system, subsystem, level, ts_ms, payload, len);  // len <= 64
```

When the logging branch merges, call this from `log_message()`. That is one line.

## For the state-machine owner

Wiring with `StateBridge`, once per control loop (order as in `systems/state/README.MD`):

```cpp
StateBridge bridge(sys_fsm, flight_fsm, &sensor_telemetry);   // also a TelemetrySource
comms_init(radio_stream, bridge, bridge);                      // setup()

// loop():
fill si / fi (sensors, fault manager, guidance; not the command fields)
bridge.set_fault_mask(active_faults);
bridge.apply(si, fi);        // ORs this loop's accepted command pulses into si / fi
sys_fsm.update(si, now);  fi.system_state = sys_fsm.state();  flight_fsm.update(fi, now);
clear the command fields (README step 6)
comms_update();
```

- A command accepted in `comms_update()` reaches the FSM on the next loop, and only for
  that loop.
- The bridge needs no FSM changes. `systems/state/` is untouched.
- Telemetry-lost fault: drive `FaultId::kTelemetryLost` from `!comms_gcs_link_ok()`,
  with its own timeout constant, rather than the shared 200 ms `debounce_ms`.
  - Ground heartbeats arrive every 500 ms and the radio can delay frames, so 200 ms
    would flap.
  - `comms_gcs_link_ok()` already uses `COMMS_GCS_TIMEOUT_MS` (3000 ms).
- When `SystemState`, `FlightState` or `FaultId` changes, the comms build fails until
  `goose.xml` and the checks in `state_bridge.cpp` are updated to match. Then run
  `uv run tools/gen_mavlink.py`.

## Generating the MAVLink headers

The C headers in `mavlink/generated/` are **not in git**. They are built from
`mavlink/goose.xml`, and the firmware will not compile without them (the error is
`mavlink/generated/goose/mavlink.h: No such file or directory`).

```
uv run tools/gen_mavlink.py
```

Run it from the repo root. The same command works on macOS, Linux and Windows.

- **Needs:** [uv](https://docs.astral.sh/uv/) and a network connection the
  first time. uv reads the dependency list at the top of the script and downloads
  pymavlink itself; nothing is installed into the project.
- **Run it:** once after cloning, and again whenever `goose.xml` changes, including
  after pulling someone else's change to it.
- **What it does:** copies `goose.xml` next to pymavlink's bundled `common.xml`, runs
  pymavlink's `mavgen` (C, MAVLink 2), and replaces `mavlink/generated/`. If generation
  fails, the existing headers are left as they were.
- **Version:** pymavlink is pinned in the dependency list at the top of the script (2.4.50). Keep
  it in step with the ground station's pymavlink.
- **Never edit the output by hand.** Change `goose.xml` and regenerate.
- **Ground station:** it does not use these C headers. It generates its own Python
  module from the same `goose.xml`, so send the author the new file after each change.

## Building and testing

```
uv run tools/gen_mavlink.py       # first, and after any goose.xml change (see above)
uv venv .venv && uv pip install --python .venv/bin/python platformio
.venv/bin/pio test -e native      # test_comms, test_radio_config, test_state_bridge (+ the owner's test_state)
.venv/bin/pio run -e teensy41     # firmware
```

**Bench test** with two radios (props off). This is a test script, not the ground
station:

```
uv run --no-project --with pymavlink==2.4.50 --with pyserial tools/comms_bench.py \
    --device /dev/tty.usbserial-XXXX --key <64 hex> [--allow-arm]
```

It sends ground heartbeats, runs each command and checks the result, changes rates,
prints telemetry with rates, and checks that unsigned and wrong-key commands get no ACK.
MAVProxy can sniff the link, and it shows `RADIO_STATUS`.

## Known limits

- One ground station and one radio link (MAVLink channel 0). Up to 8 signing streams.
- No parameter, mission or FTP protocol. `COMMAND_INT` is not supported.
- Saving to Teensy 4 EEPROM writes emulated flash and can occasionally take several
  milliseconds. The signing timestamp is saved about once a minute and on disarm. Check
  this against the loop budget (see Measurements).
- `radio_config` blocks for about 3–4 s, and only in `setup()`.
- With `COMMS_REQUIRE_SIGNING 1` and no key provisioned, the aircraft never reports the
  link as OK. Its own frames are then unsigned, and a signing ground station rejects
  them.

## Measurements (to be done on hardware, not yet run)

| Measurement | Result |
|---|---|
| Signing cost per frame on Teensy 4.1 (`micros()` around the send path) | not measured |
| `comms_update()` worst-case time per call | not measured |
| EEPROM timestamp save time | not measured |
| Sustained downlink throughput, 64 kbps air rate, ECC off, AES off | not measured |
| Same with AES on | not measured |
| Command round-trip latency (COMMAND_LONG → COMMAND_ACK), AES off / on | not measured |
| Power-cycle the Teensy: ground still accepts signed frames | not tested on hardware (passes in native test) |
