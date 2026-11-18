# Two-agent radio link test: handoff

How to pick this work back up from scratch, for a person or for a new Claude session on either machine.
Last updated 2026-11-18. Branch: `communications`.

## Update 2026-11-18 (later): two RFD900x radios, real radio link next

There are now **two** RFD900x-US radios (RFD SiK 3.57 firmware), so the RTL-SDR plan below is no
longer the main path. The plan is handoff step 4: `comms_bench.py` on the Mac ↔ `air_sim.py` on
Windows over the radio link (no Tailscale needed for the data).

| Radio | Machine | Cable | Antenna | State |
|---|---|---|---|---|
| Ground (RX) | Mac | Prolific PL2303G USB-A→4-wire (needs Prolific's "PL2303Serial" driver, installed). Port `/dev/cu.PL2303G-USBtoUART130` | 900 MHz whip | **Configured and verified** |
| Air (TX) | Windows | USB-C → 6-wire (labeled GND/CTS/VCC/TXD/RXD/RTS) | 900 MHz Yagi, via RP-SMA | **Not configured, unplugged from USB** |

**Wiring (RFD900x 2×8 header).** The row with the "1" silkscreen is odd pins 1,3,5,…,15; the other
row is 2,4,…,16. GND → pin 1, 5V → pin 4, **adapter TXD → pin 7 (radio RX), adapter RXD → pin 9
(radio TX)**: TX/RX cross over. CTS/RTS stay unconnected.

**The radios came pre-configured by someone else**: serial speed **460800**, air speed 224, NETID 25,
TX power 30 (1 W), **RTSCTS=1** (flow control on, but RTS/CTS aren't wired). At 57600 the radio looks
dead; it only answers `+++` at 460800.

**Ground radio now (saved with AT&W, read back after ATZ):** S1=57, S2=64, S3=117, S4=20 (100 mW for
bench testing), S5=0, S6=1, S8=902000, S9=915000, S13=0, S15=0, S20=0. **S9 can't be raised above
915000** (928000, 927000, 926000, 925000 all return ERROR), so both radios use 902–915 MHz, 51 channels.

**Configuring a radio:** `tools/rf/sik_config.py PORT` finds the baud (tries 57600, 460800, …) and
prints ATI5 (read-only). Add `--write` to set the values above, save, reboot and read back. Add
`--ant-mode 1` when the only antenna is on port A1 (`--ant-mode 2` for A2). For the air radio:

```powershell
.\.venv\Scripts\python.exe tools/rf/sik_config.py COM5 --ant-mode 1 --write
```

(Check the COM number first; the antenna must be on before USB goes in.) When both radios match, the
green LEDs go from blinking to solid.

**Air radio configured** the same way (S20=1, Yagi on A1). **First full run over the radio link
(2026-11-18): every check `ok`**, the same as over Tailscale. Same room, 100 mW. The ground log is in
`~/claude/rf/radio_link_run1.log` on the Mac.

| Check | Result over RF |
|---|---|
| Telemetry rates | ATTITUDE 10.1 Hz, GLOBAL_POSITION_INT / VFR_HUD 4.0, GOOSE_STATE / GPS_RAW_INT 2.0, the 1 Hz streams 1.0–1.1, all matching `comms_config.h` |
| RADIO_STATUS (injected by the radio, 2 Hz) | rssi 206/206 (saturated at this short range), noise 57/54, txbuf 100% |
| Commands, BENCH, fault, rates | Same as the Tailscale run, all `ok`. ATTITUDE at 2 Hz → 10 / 5 s, IMU log at 5 Hz → 25 / 5 s |
| Security | Unsigned and wrong-key commands get no ACK, signed ones are ACCEPTED. Signing: good 2138, bad 126 (the deliberate wrong-key phase), unsigned 352 (the radio's own RADIO_STATUS frames, which are expected unsigned) |

Commands: Mac `comms_bench.py --device /dev/cu.PL2303G-USBtoUART130 --baud 57600 --key bench-test-only
--allow-arm --link-timeout 1800` first, then Windows `air_sim.py --device COM5 --baud 57600 --key
bench-test-only --seconds 300`.

Next: separate the antennas (other rooms or outdoors) and watch rssi and noise; a range walk with the
Yagi; try `ATS4` at higher power once the radios are apart; then swap `air_sim` for the Teensy.

## Goal

Test the GOOSE comms link (MAVLink v2 with signing, see `systems/Comms_Subsystem/README.md`) without
the Teensy, with two Claude Code sessions working together:

| Machine | Role | Hardware | Claude session |
|---|---|---|---|
| **Windows PC** (`desktop-iriosde`) | Aircraft / transmit side: runs `tools/air_sim.py` | RFD900x on **COM5** (FTDI VID_0403 PID_6001, SN AB99E02P) + 900 MHz Yagi | "Hello" (Remote Control) |
| **Mac** (`cm5`) | Ground / receive side: runs `tools/comms_bench.py`, watches RF | RTL-SDR Blog V4 (R828D tuner) | "Multi-computer Claude model coordination" (Remote Control) |

There is **one** RFD900x, so the two machines can't talk over the radio. Instead:

1. **Protocol stage (done):** `air_sim.py` on Windows ↔ `comms_bench.py` on the Mac over TCP through Tailscale.
2. **RF stage (next, blocked on hardware):** Windows drives the RFD900x through `air_sim.py --device COM5`, and the Mac's RTL-SDR measures what goes on air.

## Status

| Item | State |
|---|---|
| `air_sim.py`: Python port of `comms.cpp` + stub command handler + dummy telemetry | Done, on `communications` |
| Full bench test over Tailscale (Windows sim → Mac bench) | **Passing**: every check `ok` (latest run below) |
| Mac RTL-SDR access + 902–928 MHz baseline (radio off) | Done. Baseline in `~/claude/rf/baseline.csv/.png` on the Mac |
| RF test with the RFD900x | **Blocked**: no cable between the Yagi and the RFD900x. Need the stock RFD900x antennas, an RP-SMA pigtail for the Yagi, or a dummy load |

### Latest results (Tailscale, 2026-11-18)

Two consecutive clean runs, both all `ok` (the second with the disconnect fix, 35c093c):

| Check | Result |
|---|---|
| Telemetry rates | ATTITUDE 10.1 Hz, GLOBAL_POSITION_INT / VFR_HUD 4.0, GOOSE_STATE / GPS_RAW_INT 2.0, HEARTBEAT / SYS_STATUS / BATTERY / PRESSURE / GOOSE_LOG 1.0, matching `comms_config.h` |
| Flight commands | ARM, TAKEOFF, LOITER_UNLIM, RESUME_MISSION, RTL ACCEPTED; forced DISARM in the air DENIED; DISARM after landing ACCEPTED |
| BENCH / fault | BENCH enter/exit ACCEPTED, ARM in BENCH DENIED, FAULT_CLEAR with no fault DENIED, DO_SET_MODE UNSUPPORTED |
| Rates | ATTITUDE at 2 Hz → 10–11 msgs / 5 s (11 then 10 across the two runs; the window edge decides); GET interval → 500000 us; IMU log at 5 Hz → 25 msgs / 5 s |
| Security | Unsigned and wrong-key commands get no ACK; signed control ACCEPTED. Signing 2143 good / 123 bad (bad = only the deliberate wrong-key phase) |

Measured aircraft downlink at the default rates: **~1.5 kB/s** (budget 3 kB/s; 57600 baud ≈ 5.7 kB/s).
This is the first measurement of the README's estimate.

## Safety rules (both agents)

- **Never power the RFD900x without an antenna or dummy load attached.** A SiK radio can transmit sync
  packets as soon as it's powered (up to 1 W). Keep it **unplugged from USB** until an antenna is on.
- Don't open COM5 or transmit until the user explicitly OKs the RF stage.
- For a test in the same room, keep the antennas a few metres apart, or lower TX power (`ATS4`): the Yagi
  focuses power forward and can overload a nearby receiver.
- Stay in 902–928 MHz (US ISM). The RFD900x-US firmware already does.

## Network between the machines

- Path: **Tailscale**. Mac `100.85.138.126`, Windows `100.84.65.42`. It currently goes through the
  DERP relay `sea` (~5 ms, no direct path), which is fine.
- The **Mac listens** (`tcpin:0.0.0.0:5760`) and **Windows connects out** (`tcp:100.85.138.126:5760`),
  because the Windows firewall blocks inbound.
- **ProtonVPN on Windows blocks the connection** (`WinError 10013`). Turn it fully off before a linked
  run. Only "fully off" has been tested; excluding Tailscale with split tunnelling might work but is
  untested.
- pymavlink's `tcpin` accepts one client at a time, so don't probe the port with `Test-NetConnection`
  while a run is set up. A probe that connects and drops can trigger the bench early (that happened
  once: the bench ran its commands against nobody, then the real sim joined halfway through).

## Running the protocol test

Key: the throwaway passphrase `bench-test-only` (fingerprint `bc1fd57beea0`). Never put a real flight
key in git or in chat.

**1. Mac (ground) first.** It listens and waits up to 30 minutes for the aircraft:

```bash
cd ~/Documents/UW_Work/AFC_Drone/Autonomous-Controls-Fixed-Wing
uv run --no-project --with pymavlink==2.4.50 --with pyserial tools/comms_bench.py \
    --device tcpin:0.0.0.0:5760 --key bench-test-only --allow-arm --link-timeout 1800
```

**2. Windows (aircraft) second:**

```powershell
.\.venv\Scripts\python.exe -u tools/air_sim.py --device tcp:100.85.138.126:5760 --key bench-test-only --seconds 150
```

Start it **once**, after the Mac listener is up. A run that dies partway still counts as a connection,
and the bench then runs its sequence against nobody. About 20 lines of mavgen chatter at startup
(`Validation skipped`, `Parsing`, `CAMERA_TRACKING_STATUS_FLAGS_IDLE / MAV_BOOL_FALSE has invalid
values`, …) are harmless; `comms_bench.py` prints the same.

The full run takes ~90 s: telemetry for 10 s, then ARM → TAKEOFF → (CLIMB, CRUISE) → LOITER → RESUME → RTL →
DISARM-in-air (DENIED) → landing → DISARM → BENCH tests → rate changes → security (unsigned and
wrong-key commands must get no ACK). Every line should end in `ok`. When the bench finishes and closes,
`air_sim` prints `link closed (...); exiting`.

**Without the network** (one machine), the same test can run with both programs in one process over a
socketpair. That's useful when a sandbox blocks ports. Ask the Mac session for its `loopback_test.py`
harness, or recreate it: patch `mavutil.mavlink_connection` to return a `mavfile` wrapping each end
of `socket.socketpair()`, then run `air_sim.Aircraft(...).run()` in a thread and `comms_bench.main()`.

## Machine setup notes

### Mac

- Repo: `~/Documents/UW_Work/AFC_Drone/Autonomous-Controls-Fixed-Wing`. Claude session folder: `~/claude`.
- **Claude Code sandbox is OFF** for the `~/claude` project (`~/claude/.claude/settings.local.json` →
  `"sandbox": {"enabled": false}`). That was needed for USB (RTL-SDR), listening on 5760, and git push
  over SSH. `excludedCommands` and `allowLocalBinding` alone did **not** give USB access. Re-enable the
  sandbox when the SDR isn't needed.
- RTL-SDR: Homebrew `librtlsdr` 2.0.3 (supports the Blog V4). Check with `rtl_test -t`.
  Baseline sweep:
  ```bash
  rtl_power -f 902M:928M:25k -g 30 -i 1 -e 30s baseline.csv
  uv run --no-project --with numpy --with matplotlib tools/rf/rtl_power_plot.py baseline.csv baseline.png
  ```
  The baseline shows ~230 short bursts in 30 s from other 915 MHz devices (meters, LoRa, …), up to 30 dB
  over the floor. The dark vertical stripes every 2.6 MHz are rtl_power tuning-edge rolloff, not signals.
  So finding the RFD900x means comparing radio on vs off, its hop pattern (50 channels by default) and
  its TDM rhythm, not just "is there energy".

### Windows

- Windows 10 Pro 10.0.19045. Run everything from the repo root in PowerShell, on branch `communications`.
- **Python:** system Python 3.9.9. Pinned packages: pymavlink 2.4.50 (pulls in lxml) and pyserial 3.5.
- **uv doesn't work here for pymavlink.** `uv run --with pymavlink==2.4.50 …` fails every time with
  `failed to remove directory …\uv\cache\builds-v0\.tmpXXXX\…pymavlink-2.4.50.data … being used by
  another process (os error 32)`, probably antivirus scanning new files. Retrying, `--python 3.12` and
  `UV_LINK_MODE=copy` didn't help. **Use a plain venv in the repo instead** (`.venv` is gitignored):
  ```powershell
  python -m venv .venv
  .\.venv\Scripts\python.exe -m pip install pymavlink==2.4.50 pyserial==3.5
  ```
  This repo `.venv` already exists on the Windows PC (Python 3.9.9, pymavlink 2.4.50, pyserial 3.5,
  lxml 6.1.3; the goose dialect loads). The two passing runs used an older venv in a Claude scratchpad, so
  the first full run from the repo `.venv` is still to be done.
- `python` on PATH resolves to a Python 3.9 install under a *different* Windows user profile. It works,
  but calling `.\.venv\Scripts\python.exe` directly avoids surprises.
- **ProtonVPN must be fully off.** Its `ProTUN` adapter blocks outbound traffic to the Tailscale peer:
  Python gets `WinError 10013` on connect. Quick check that raw sockets get out: a TCP connect to an
  unused port on the Mac should fail with **10061** (refused), not **10013** (forbidden).
- Windows Firewall blocks inbound (Ethernet is on the Public profile). The design avoids that: Windows
  only connects out.
- **Claude Code sandbox is OFF** for the Windows project: `.claude/settings.local.json` →
  `{"sandbox": {"enabled": false}}`. The Windows session also added `.claude/settings.local.json` to
  `.gitignore`; that edit is **not committed yet** (waiting on the user). Delete the file, or set
  `enabled` to true, to turn the sandbox back on.
- **Serial ports:** RFD900x = **COM5** (FTDI USB Serial, VID_0403 PID_6001, SN AB99E02P), 57600 baud.
  COM1 is the onboard port; COM3/COM4 are Bluetooth serial links, not the radio.
- **Antenna:** none available yet. The RFD900x ports are **RP-SMA**; the Yagi's coax ends in a different
  connector (probably N-type), so it needs an N → RP-SMA-male pigtail. Alternatives: the stock RFD900x
  antennas, or a 50 Ω dummy load / attenuator rated ≥ 1 W. USB alone powers the radio. **Order for the RF
  test: attach the antenna or load → plug in USB → run air_sim with `--device COM5`.** Consider lowering TX
  power (`ATS4`) first, with the user's OK.

## Next steps

1. **Get an antenna on the RFD900x**: stock antennas, an RP-SMA pigtail matching the Yagi's connector, or a
   dummy load + attenuator.
2. **RF stage**, coordinated between the agents:
   - Windows: `air_sim.py --device COM5 --baud 57600 --key bench-test-only` (no ground radio, so no link; it
     just transmits telemetry). Confirm first that the radio's registers match `README.md` (NETID 117,
     AIR_SPEED 64, MAVLINK 1).
   - Mac: `rtl_power` sweeps with the radio on vs off, then a narrower `rtl_sdr` IQ capture on a few hop
     channels to see burst timing. Raise/lower `air_sim` stream rates and check the airtime follows.
   - Rotate the Yagi to check its directivity.
3. Stretch: demodulate SiK GFSK packets from IQ (hard: frequency hopping + framing).
4. When a second RFD900x is available: run `comms_bench.py` on the Mac over a real radio link against
   `air_sim.py` (or the Teensy) on the other end.

## Starting tomorrow (new Claude sessions)

1. On each machine, open Claude Code in the folder above, turn on **Remote Control**, and give the
   session a clear name ("Mac RX", "Windows TX").
2. Tell each session to read `tools/LINK_TEST_HANDOFF.md` on branch `communications`.
3. The sessions find each other with `ListAgents` and message with `SendMessage`.
4. Before any linked run: Tailscale up on both machines (`tailscale status` shows the other as active);
   ProtonVPN fully off on Windows; on the Mac the sandbox is off (or allows binding); the Mac listener
   starts first.
