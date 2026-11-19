# Windows laptop setup for the radio tests

Use this to run the **air side** of the range test on a Windows laptop: the Yagi radio driven by
`tools/air_sim.py`. It assumes Git and Python 3 are already installed. The radio is already wired and
configured, so leave its wiring and settings alone.

Run everything in **PowerShell**.

## 1. Get the code and the Python packages (one time)

```powershell
git clone https://github.com/Autonomous-Flight-Team/Autonomous-Controls-Fixed-Wing.git
cd Autonomous-Controls-Fixed-Wing
git checkout communications
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install pymavlink==2.4.50 pyserial==3.5
```

If you already have the repo, run `git checkout communications` and `git pull` instead of the clone.

## 2. Plug in the radio and find its COM port

Make sure the Yagi is connected before the USB cable goes in. Then:

```powershell
Get-PnpDevice -Class Ports -PresentOnly | Format-Table FriendlyName
```

The radio shows up as something like **`USB Serial Port (COM5)`**. Use that COM number below.

## 3. Run it

```powershell
.\.venv\Scripts\python.exe -u tools/air_sim.py --device COM5 --baud 57600 --key bench-test-only --seconds 0
```

It runs until **Ctrl+C**. About 20 lines of `Validation skipped` / `Parsing` / `invalid values` at startup
are normal. Once the ground side is running, it prints `ground station link UP`, and the radio's green
LED goes solid.

## If something goes wrong

| Symptom | Fix |
|---|---|
| No `USB Serial Port` in step 2 | Windows hasn't installed the cable's driver. Install the FTDI "VCP" driver from ftdichip.com, then replug. |
| `could not open port 'COM5'` | Something else has the port open (another terminal or a ground-station app). Close it, or replug the cable. |
| Never prints `link UP` | Start the ground side, and check both sides use `--key bench-test-only` (fingerprint `bc1fd57beea0`). |
| Want to check the radio's settings | `.\.venv\Scripts\python.exe tools/rf/sik_config.py COM5` only reads them. They should show S1=57, S2=64, S3=117, S4=20, S20=1. |
