# RC Link

## Overview

Reads the pilot's ExpressLRS receiver (RadioMaster ER8) over CRSF and fills the
`PilotInput` / `RcLinkStatus` structs from `systems/pilot/pilot_input.h`. It
reports facts only: `pilot/` decides `kRcLost`, manual control and mirroring.

- **`rc_link.h/.cpp`**: frame parser, channel mapping, link statistics. No
  Arduino code (it reads a `ByteStream`), so it is tested on native.

## Wiring

ER8 CRSF TX → Teensy RX2 (pin 7), ER8 RX ← Teensy TX2 (pin 8, unused for now),
GND common. The receiver's serial protocol must be CRSF, not PWM.

```cpp
// setup()
static uint8_t rc_rx_buf[RC_LINK_RX_EXTRA_BUFFER];
Serial2.begin(RC_LINK_BAUD);                          // 420000
Serial2.addMemoryForRead(rc_rx_buf, sizeof(rc_rx_buf));
static ArduinoByteStream rc_stream(Serial2);
rc_link_init(rc_stream);                              // default RcLinkConfig

// loop(), first thing
rc_link_update(now_ms);
PilotInput rc;
rc_link_latest(&rc);
```

The extra read buffer matters: Teensy's default is 64 bytes, a few
milliseconds at 420000 baud. At a 500 Hz packet rate a 20 ms loop receives
about 600 bytes.

## What it parses

| Frame | Type | Result |
|---|---|---|
| RC channels packed | `0x16` | 16 channels, ticks → µs with `crsf_ticks_to_us()` (pilot.h, same as ELRS), mapped to roll/pitch/yaw/throttle/flaps and switch bits; `stamp_ms`, `frames_seen`, `last_frame_ms` |
| Link statistics | `0x14` | `link_quality` (uplink LQ %), `rssi_dbm` (active antenna), `stats_seen` |
| anything else | | valid CRC → ignored |

- Frames start with sync `0xC8`. A bad CRC or impossible length is dropped,
  counted in `bad_frames`, and the parser resyncs on the next `0xC8`.
- The newest RC frame wins; older frames in the same loop are overwritten.
- There is no failsafe flag: the ER8 stops sending RC frames when it loses the
  transmitter, so the frame age is what `rc_lost_raw()` / `pilot_fresh()` use.
- `link_quality` is unknown until the first link statistics frame:
  `stats_seen` stays false, `rc_lost_raw()` judges by frame age alone, and
  telemetry reports LQ 255 (unknown). The ER8 sends statistics about every
  100 ms, so the first RC frames arrive before the first statistics frame.

## Channel map (`RcLinkConfig`)

Defaults are the ELRS/EdgeTX AETR order. Change them to match the TX16S model.

| Field | Default | Scaling |
|---|---|---|
| `roll_ch` | CH1 | 988..2012 µs → −1..1 (1500 = 0) |
| `pitch_ch` | CH2 | same |
| `throttle_ch` | CH3 | 988..2012 µs → 0..1 |
| `yaw_ch` | CH4 | −1..1 |
| `flaps_ch` | none | 0..1 |
| `bypass_ch` | CH7 (SA) | `PilotSwitch::kBypass` when ≥ `switch_on_us` (1750) |
| `autonomy_ch` | none | `PilotSwitch::kAutonomy` |

Reversing and end points belong on the transmitter. `raw_us` is never
rescaled, so mirroring stays exact.

## Changes made on manual_controls

**2026-11-19, `RcLinkStatus::stats_seen`** (agreed with Advik; owner of
`pilot_input.h` is manual controls):

- `pilot_input.h`: new `bool stats_seen` in `RcLinkStatus`.
- `rc_link.cpp`: `on_link_stats()` sets `g.status.stats_seen = true` (one
  line). Nothing else in rc_link changed.
- `pilot.cpp`: `rc_lost_raw()` only checks `link_quality` once `stats_seen`
  is true, and `rc_link_quality_pct()` returns 255 (unknown) until then
  (before: `link_quality` read 0, which counted as RC lost until the first
  statistics frame).
- Tests: `test_rc_link` gained `test_rc_frames_before_stats_are_not_lost`.

If you work on the `rc_link` branch again, take this commit from
`manual_controls` (merge or cherry-pick) or make the same one-line change in
`on_link_stats()`: an identical edit merges cleanly. CRC failures still drop
the whole frame; that's intended (simple over saving one frame).

## Tests

`test/test_rc_link`: CRC check value, channel packing round trip, the
PilotInput it builds (through pilot.h's helpers), link statistics, bad CRC,
noise and split frames, ignored frame types, and a 500 Hz burst.
