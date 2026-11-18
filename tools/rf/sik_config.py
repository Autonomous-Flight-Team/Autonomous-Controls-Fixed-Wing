"""Configure an RFD900x (SiK) for the GOOSE link test.

Usage: python sik_config.py PORT [--ant-mode N] [--write]
Finds the radio's current baud, prints ATI5, and with --write sets the project values,
saves (AT&W) and reboots (ATZ). Then re-reads at 57600 to confirm.
"""
import argparse, sys, time, serial

BAUDS = [57600, 460800, 115200, 230400, 38400, 19200, 9600, 4800, 2400, 1200]
SETTINGS = [
    ("S1", 57),      # serial speed 57600
    ("S2", 64),      # air speed 64 kbps
    ("S3", 117),     # NETID
    ("S4", 20),      # TX power 20 dBm (100 mW) for bench testing
    ("S5", 0),       # ECC off
    ("S6", 1),       # MAVLink framing
    ("S8", 902000),  # min freq (max stays 915000: this firmware rejects higher)
    ("S13", 0),      # RTS/CTS off (not wired)
    ("S15", 0),      # encryption off
]


def enter_cmd(port):
    for b in BAUDS:
        s = serial.Serial(port, b, timeout=0.3)
        time.sleep(1.3)
        s.reset_input_buffer()
        s.write(b"+++")
        time.sleep(1.5)
        if b"OK" in s.read(4096):
            return s, b
        s.close()
    sys.exit("radio did not answer +++ at any baud")


def at(s, c, wait=0.6):
    s.reset_input_buffer()
    s.write((c + "\r\n").encode())
    time.sleep(wait)
    return s.read(s.in_waiting or 1).decode(errors="replace").replace(c, "", 1).strip()


ap = argparse.ArgumentParser()
ap.add_argument("port")
ap.add_argument("--ant-mode", type=int, default=0)
ap.add_argument("--write", action="store_true")
a = ap.parse_args()

s, baud = enter_cmd(a.port)
print(f"radio answered at {baud}: {at(s, 'ATI')}")
if not a.write:
    print(at(s, "ATI5", 1.0))
    at(s, "ATO")
    sys.exit(0)

for reg, val in SETTINGS + [("S20", a.ant_mode)]:
    r = at(s, f"AT{reg}={val}")
    print(f"AT{reg}={val} -> {r}")
    if "OK" not in r:
        sys.exit(f"setting {reg} failed; nothing saved (radio still has old values after a reboot)")
print("AT&W ->", at(s, "AT&W", 1.0))
s.write(b"ATZ\r\n")
time.sleep(0.3)
s.close()
time.sleep(4)

s, baud = enter_cmd(a.port)
print(f"after reboot, radio answers at {baud}")
print(at(s, "ATI5", 1.0))
at(s, "ATO")
