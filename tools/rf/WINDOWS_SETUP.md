# Windows laptop setup for the radio tests

These steps set up a Windows 10 or 11 laptop to run the **air side** of the radio link test: the RFD900x
with the Yagi, driven by `tools/air_sim.py`. The same setup also runs the ground-side tools
(`comms_bench.py`, `rf/link_logger.py`) if a Windows laptop has to be the ground station.

Allow about 15 minutes. You need internet for the install, but not for the test itself.

## What you need

- A Windows laptop with a free USB port. Some steps need admin rights (installing Python, Git and maybe a driver).
- The **air radio**: the RFD900x-US marked for the Yagi, the Yagi with its RP-SMA cable, and the
  **USB-C → 6-wire cable** (wires labeled GND, CTS, VCC, TXD, RXD, RTS).
- A USB-C port on the laptop, or a USB-C → USB-A adapter.

## 1. Install Python and Git

Open **PowerShell** (Start → type "PowerShell") and run:

```powershell
winget install -e --id Python.Python.3.12
winget install -e --id Git.Git
```

Close PowerShell and open a new one, so it picks up the new PATH. Then check:

```powershell
py -3.12 --version
git --version
```

If `winget` isn't available: install Python 3.12 from python.org (tick **"Add python.exe to PATH"**) and
Git from git-scm.com, keeping the defaults.

## 2. Get the code

```powershell
cd $HOME\Documents
git clone https://github.com/Autonomous-Flight-Team/Autonomous-Controls-Fixed-Wing.git
cd Autonomous-Controls-Fixed-Wing
git checkout communications
```

The repo may be private. If so, Git asks you to sign in to GitHub in a browser window. Your account needs
access to the Autonomous-Flight-Team organisation. If Git isn't an option, download the
`communications` branch as a ZIP from GitHub (Code → Download ZIP), unzip it, and `cd` into that folder.

**All the commands below run from this repo folder.**

## 3. Create the Python environment

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install pymavlink==2.4.50 pyserial==3.5
```

Notes:
- Call `.\.venv\Scripts\python.exe` directly, as shown. Then you never need to "activate" the venv, and
  PowerShell's script-execution policy doesn't get in the way.
- Don't use `uv` on Windows for this. It failed on the lab desktop: pymavlink's install files were locked
  by another process, probably antivirus. A plain venv works.

Check that the GOOSE MAVLink dialect builds. This is the step that fails if something is missing:

```powershell
.\.venv\Scripts\python.exe -c "import sys; sys.path.insert(0,'tools'); import comms_bench; comms_bench.load_goose_dialect(); print('dialect OK')"
```

About 20 lines of `Validation skipped` / `Parsing` / `has invalid values` messages are normal. The last
line should be `dialect OK`.

## 4. Wire and connect the radio

**Safety: the antenna goes on before the radio gets power.** A powered RFD900x can transmit at up to
1 W, and with no antenna on its port that can damage the radio.

1. Connect the Yagi to the radio's **A1** port. A1 and A2 are marked on the board next to the gold connectors.
2. Check the wiring on the radio's 2×8 header. The row printed with **"1"** holds pins 1, 3, 5 … 15, and the
   other row holds 2, 4 … 16. **TX and RX cross over:**

   | Cable wire | Radio pin |
   |---|---|
   | GND | 1 |
   | VCC | 4 |
   | TXD | **7** (radio RX) |
   | RXD | **9** (radio TX) |
   | CTS, RTS | not connected |

3. Plug the cable into the laptop. The radio's green LED should start blinking. That means it's searching
   for the other radio, and it goes solid once they pair.

## 5. Find the COM port

```powershell
Get-PnpDevice -Class Ports -PresentOnly | Format-Table FriendlyName, Status
```

Look for something like `USB Serial Port (COM5)`. That's an FTDI chip, which the labeled USB-C cable
uses. You can also find it in Device Manager → **Ports (COM & LPT)**. Unplug and replug the cable to see
which entry it is. COM1 and the Bluetooth COM ports are never the radio.

If nothing appears, or the device shows a yellow warning icon, Windows needs the driver:
- **FTDI cable** (usually installs on its own through Windows Update): get the "VCP" driver from
  ftdichip.com → Drivers → VCP.
- **Prolific cable** (the unlabeled 4-wire red, black, green and white one): get the PL2303 driver from
  prolific.com.tw. Its wires are red = 5V (pin 4), black = GND (pin 1), green and white = data on pins 7
  and 9. If the radio doesn't answer, swap green and white.

Below, replace `COM5` with your port.

## 6. Check the radio settings

This only reads the settings. It works out the radio's baud rate by itself.

```powershell
.\.venv\Scripts\python.exe tools/rf/sik_config.py COM5
```

The radios used so far are already set up. They should show:

| Setting | Value |
|---|---|
| S1 serial speed | 57 |
| S2 air speed | 64 |
| S3 network ID | 117 |
| S4 TX power | 20 |
| S5 ECC | 0 |
| S6 MAVLink | 1 |
| S8 min frequency | 902000 |
| S9 max frequency | 915000 |
| S13 RTS/CTS | 0 |
| S15 encryption | 0 |
| S20 antenna mode | 1 (Yagi on A1) |

If the values are different (for example, a new radio still at the factory settings or at 460800 baud),
write them:

```powershell
.\.venv\Scripts\python.exe tools/rf/sik_config.py COM5 --ant-mode 1 --write
```

Both radios must match on everything except S20, or they won't pair. **Don't set S4 above 29** with the
Yagi: 30 (1 W) plus the Yagi's ~7 dBi is over the FCC limit.

## 7. Run the air side

First check that the ground station is ready. Then:

```powershell
.\.venv\Scripts\python.exe -u tools/air_sim.py --device COM5 --baud 57600 --key bench-test-only --seconds 0
```

`--seconds 0` runs until you press **Ctrl+C**, which is what you want for a range test. For the scripted
bench test (`comms_bench.py`), use `--seconds 300`. `bench-test-only` is the throwaway test key. Never
put a real flight key in a command you paste into chat or commit to the repo.

The startup messages include the mavgen chatter, then `link UP` once ground heartbeats arrive.

## Using a Windows laptop as the ground station instead

The steps are the same, with the ground radio (the 900 MHz whip antenna) on its COM port:

```powershell
.\.venv\Scripts\python.exe -u tools/rf/link_logger.py --device COM6 --key bench-test-only
```

The dashboard opens in the browser at http://127.0.0.1:8765. The first time, Windows Firewall may ask
about Python; **Private networks** is enough. Log files go in `range_logs\`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `could not open port 'COM5': PermissionError` | Something else has the port open: another terminal, Mission Planner, QGroundControl, or a run still going. Close it, or unplug and replug the cable. |
| `sik_config.py`: "radio did not answer +++" | TX and RX on the wrong pins: they cross over, see step 4. Check the cable is in the right row of the header, and that the green LED is on. |
| Green LED never goes solid | The radios' settings don't match. Run step 6 on both radios and compare. Also check the other radio is powered. |
| `air_sim` never prints `link UP` | The ground side isn't running, or uses a different `--key`. Both sides print a key fingerprint; `bench-test-only` gives `bc1fd57beea0`. |
| `py` not found | Python's PATH wasn't added. Reinstall Python with the PATH box ticked, or use the full path to `python.exe`. |
| pip fails building `lxml` | Make sure it's Python 3.12, which has ready-made lxml wheels. Very new Python versions may not have them yet. |
