#!/usr/bin/env python3
"""GOOSE comms bench test (NOT the ground station).

Talks to the aircraft through the ground RFD900x with pymavlink: sends ground
heartbeats, exercises every uplink command, changes stream rates, prints the
telemetry it receives, and checks that unsigned and wrong-key commands are ignored.

Run with uv from the repo root (props OFF, aircraft on the bench):

    uv run --no-project --with pymavlink==2.4.50 --with pyserial \\
        tools/comms_bench.py --device /dev/tty.usbserial-XXXX --key <64 hex chars>

The key must be the one provisioned on the Teensy with `KEY <hex>`. Never commit a
real key; for a desk test with a throwaway key use e.g. --key "bench-test-only".
A key that is not 64 hex characters is treated as a passphrase: key = SHA-256(text),
which is also what MAVProxy's `signing setup <passphrase>` does.
"""
import argparse
import hashlib
import os
import shutil
import sys
import tempfile
import time

os.environ["MAVLINK20"] = "1"  # must be set before importing mavutil

import pymavlink  # noqa: E402
from pymavlink import mavutil  # noqa: E402
from pymavlink.generator import mavgen  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GOOSE_XML = os.path.join(REPO, "systems", "Comms_Subsystem", "mavlink", "goose.xml")
FORCE_MAGIC = 21196


def load_goose_dialect():
    """Generate the Python module for goose.xml (with common.xml) and make mavutil use it."""
    work = tempfile.mkdtemp(prefix="goose_dialect_")
    defs = os.path.join(os.path.dirname(pymavlink.__file__), "message_definitions", "v1.0")
    for name in os.listdir(defs):
        if name.endswith(".xml"):
            shutil.copy(os.path.join(defs, name), work)
    shutil.copy(GOOSE_XML, work)
    out = os.path.join(work, "goose.py")
    mavgen.mavgen(mavgen.Opts(output=out, wire_protocol="2.0", language="Python3", validate=False),
                  [os.path.join(work, "goose.xml")])
    sys.path.insert(0, work)
    import goose  # noqa: E402  (generated above)
    sys.path.remove(work)
    shutil.rmtree(work, ignore_errors=True)
    mavutil.mavlink = goose
    mavutil.current_dialect = "goose"
    return goose


def parse_key(text):
    try:
        key = bytes.fromhex(text)
        if len(key) == 32:
            return key
    except ValueError:
        pass
    return hashlib.sha256(text.encode()).digest()


def enum_name(mav, enum, value):
    entry = mav.enums.get(enum, {}).get(value)
    return entry.name.replace(enum + "_", "") if entry else str(value)


class Bench:
    def __init__(self, args, mav):
        self.args = args
        self.mav = mav
        self.conn = mavutil.mavlink_connection(args.device, baud=args.baud, source_system=args.sysid,
                                               source_component=mav.MAV_COMP_ID_MISSIONPLANNER)
        self.key = parse_key(args.key)
        # Accept the radio's unsigned RADIO_STATUS; everything else must be signed.
        self.conn.setup_signing(self.key, sign_outgoing=True,
                                allow_unsigned_callback=lambda _mav, msg_id: msg_id == mav.MAVLINK_MSG_ID_RADIO_STATUS)
        self.next_hb = 0.0
        self.counts = {}
        self.latest = {}
        print(f"key fingerprint {hashlib.sha256(self.key).hexdigest()[:12]} (compare with KEY? on the Teensy)")

    # -- link plumbing
    def pump(self, seconds):
        """Receive for `seconds`, sending a 2 Hz ground heartbeat. Returns messages seen."""
        seen = []
        end = time.time() + seconds
        while time.time() < end:
            if time.time() >= self.next_hb:
                self.conn.mav.heartbeat_send(self.mav.MAV_TYPE_GCS, self.mav.MAV_AUTOPILOT_INVALID, 0, 0,
                                             self.mav.MAV_STATE_ACTIVE)
                self.next_hb = time.time() + 0.5
            m = self.conn.recv_match(blocking=True, timeout=0.05)
            if m is None or m.get_type() == "BAD_DATA":
                continue
            t = m.get_type()
            self.counts[t] = self.counts.get(t, 0) + 1
            self.latest[t] = m
            seen.append(m)
            if t == "STATUSTEXT":
                print(f"  STATUSTEXT [{m.severity}] {m.text}")
        return seen

    def command(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, wait=1.5, retries=2):
        """COMMAND_LONG with the standard retry scheme (confirmation counts up). Returns MAV_RESULT or None."""
        for confirmation in range(retries + 1):
            self.conn.mav.command_long_send(self.args.target, 1, cmd, confirmation, p1, p2, p3, p4, p5, p6, p7)
            for m in self.pump(wait):
                if m.get_type() == "COMMAND_ACK" and m.command == cmd:
                    return m.result
        return None

    def show_command(self, label, cmd, *params, expect=None):
        r = self.command(cmd, *params)
        name = "no ACK" if r is None else enum_name(self.mav, "MAV_RESULT", r)
        ok = "" if expect is None else ("  ok" if r == expect else f"  UNEXPECTED (wanted {expect})")
        print(f"{label:<34} -> {name}{ok}")
        return r

    # -- test steps
    def wait_for_link(self):
        print("waiting for aircraft heartbeat ...")
        for _ in range(20):
            for m in self.pump(0.5):
                if m.get_type() == "HEARTBEAT" and m.get_srcSystem() == self.args.target:
                    print(f"aircraft sysid {self.args.target} up, signing stats: good {self.conn.mav.signing.goodsig_count}"
                          f" bad {self.conn.mav.signing.badsig_count}")
                    return True
        print("no signed heartbeat from the aircraft (wrong key? wrong port? radios paired?)")
        return False

    def commands(self):
        """Walks the stub handler (or the real FSM via StateBridge) through every command."""
        m = self.mav
        print("\n== commands")
        if self.args.allow_arm:
            self.show_command("ARM", m.MAV_CMD_COMPONENT_ARM_DISARM, 1, expect=m.MAV_RESULT_ACCEPTED)
            self.show_command("TAKEOFF", m.MAV_CMD_NAV_TAKEOFF, 10, 0, 0, 0, 0, 0, 50,
                              expect=m.MAV_RESULT_ACCEPTED)
            print("waiting 11 s for the dummy flight to reach CRUISE ...")
            self.pump(11)
            self.show_command("LOITER_UNLIM", m.MAV_CMD_NAV_LOITER_UNLIM, expect=m.MAV_RESULT_ACCEPTED)
            self.show_command("GOOSE_RESUME_MISSION", m.MAV_CMD_GOOSE_RESUME_MISSION, expect=m.MAV_RESULT_ACCEPTED)
            self.show_command("RTL", m.MAV_CMD_NAV_RETURN_TO_LAUNCH, expect=m.MAV_RESULT_ACCEPTED)
            self.show_command("DISARM while airborne (forced)", m.MAV_CMD_COMPONENT_ARM_DISARM, 0, FORCE_MAGIC,
                              expect=m.MAV_RESULT_DENIED)
            print("waiting 16 s for the dummy landing (RTL -> DESCENT -> LANDING -> LANDED) ...")
            self.pump(16)
            self.show_command("DISARM on the ground", m.MAV_CMD_COMPONENT_ARM_DISARM, 0, expect=m.MAV_RESULT_ACCEPTED)
        else:
            print("ARM/TAKEOFF skipped (pass --allow-arm on the bench with props off)")
            self.show_command("RTL on the ground", m.MAV_CMD_NAV_RETURN_TO_LAUNCH, expect=m.MAV_RESULT_DENIED)
        self.show_command("GOOSE_BENCH enter", m.MAV_CMD_GOOSE_BENCH, 1, expect=m.MAV_RESULT_ACCEPTED)
        self.show_command("ARM while in BENCH", m.MAV_CMD_COMPONENT_ARM_DISARM, 1, expect=m.MAV_RESULT_DENIED)
        self.show_command("GOOSE_BENCH exit (-> PREFLIGHT)", m.MAV_CMD_GOOSE_BENCH, 0, expect=m.MAV_RESULT_ACCEPTED)
        self.show_command("GOOSE_FAULT_CLEAR with no fault", m.MAV_CMD_GOOSE_FAULT_CLEAR, expect=m.MAV_RESULT_DENIED)
        self.show_command("unsupported (DO_SET_MODE)", m.MAV_CMD_DO_SET_MODE, expect=m.MAV_RESULT_UNSUPPORTED)

    def rates(self):
        m = self.mav
        print("\n== rates")
        self.show_command("ATTITUDE at 2 Hz", m.MAV_CMD_SET_MESSAGE_INTERVAL, m.MAVLINK_MSG_ID_ATTITUDE, 500000,
                          expect=m.MAV_RESULT_ACCEPTED)
        self.counts.clear()
        self.pump(5)
        print(f"  ATTITUDE received in 5 s: {self.counts.get('ATTITUDE', 0)} (expect ~10)")
        self.show_command("GET ATTITUDE interval", m.MAV_CMD_GET_MESSAGE_INTERVAL, m.MAVLINK_MSG_ID_ATTITUDE)
        mi = self.latest.get("MESSAGE_INTERVAL")
        if mi:
            print(f"  MESSAGE_INTERVAL id {mi.message_id}: {mi.interval_us} us")
        self.show_command("ATTITUDE back to default", m.MAV_CMD_SET_MESSAGE_INTERVAL, m.MAVLINK_MSG_ID_ATTITUDE, 0,
                          expect=m.MAV_RESULT_ACCEPTED)
        self.show_command("IMU log stream at 5 Hz", m.MAV_CMD_GOOSE_SET_LOG_RATE, 0x00, 0x01, 200000,
                          expect=m.MAV_RESULT_ACCEPTED)
        self.counts.clear()
        self.pump(5)
        print(f"  GOOSE_LOG received in 5 s: {self.counts.get('GOOSE_LOG', 0)} (expect ~25)")
        self.show_command("IMU log stream default", m.MAV_CMD_GOOSE_SET_LOG_RATE, 0x00, 0x01, 0)

    def telemetry(self, seconds):
        print(f"\n== telemetry for {seconds} s")
        self.counts.clear()
        self.pump(seconds)
        for t in sorted(self.counts):
            print(f"  {t:<22} {self.counts[t] / seconds:5.1f} Hz")
        gs = self.latest.get("GOOSE_STATE")
        if gs:
            m = self.mav
            faults = [e.name.replace("GOOSE_FAULT_", "") for v, e in m.enums["GOOSE_FAULT"].items()
                      if v < (1 << 31) and gs.active_faults & v and "ENUM_END" not in e.name]
            flags = [e.name.replace("GOOSE_STATE_FLAGS_", "") for v, e in m.enums["GOOSE_STATE_FLAGS"].items()
                     if gs.flags & v and "ENUM_END" not in e.name]
            print(f"  GOOSE_STATE system {enum_name(m, 'GOOSE_SYSTEM_STATE', gs.system_state)}"
                  f" flight {enum_name(m, 'GOOSE_FLIGHT_STATE', gs.flight_state)}"
                  f" flags {flags or '-'} faults {faults or '-'} crit_entries {gs.critical_entries}"
                  f" sensors {list(gs.sensor_status)}")
        rs = self.latest.get("RADIO_STATUS")
        if rs:
            print(f"  RADIO_STATUS rssi {rs.rssi}/{rs.remrssi} noise {rs.noise}/{rs.remnoise} txbuf {rs.txbuf}%")
        log = self.latest.get("GOOSE_LOG")
        if log:
            print(f"  GOOSE_LOG sys 0x{log.system:02x} sub 0x{log.subsystem:02x} level 0x{log.level:02x}"
                  f" payload {bytes(log.payload[:log.len]).hex()}")

    def security(self):
        m = self.mav
        print("\n== security (each should get NO ACK)")
        signing = self.conn.mav.signing
        signing.sign_outgoing = False
        self.show_command("unsigned GET_MESSAGE_INTERVAL", m.MAV_CMD_GET_MESSAGE_INTERVAL, m.MAVLINK_MSG_ID_ATTITUDE,
                          expect=None)
        signing.sign_outgoing = True
        good = signing.secret_key
        signing.secret_key = hashlib.sha256(b"definitely the wrong key").digest()
        self.show_command("wrong-key GET_MESSAGE_INTERVAL", m.MAV_CMD_GET_MESSAGE_INTERVAL, m.MAVLINK_MSG_ID_ATTITUDE,
                          expect=None)
        signing.secret_key = good
        self.show_command("correctly signed (control)", m.MAV_CMD_GET_MESSAGE_INTERVAL, m.MAVLINK_MSG_ID_ATTITUDE,
                          expect=m.MAV_RESULT_ACCEPTED)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", required=True, help="ground radio serial port, or e.g. udpin:0.0.0.0:14550")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--key", required=True, help="signing key: 64 hex chars, or a passphrase (SHA-256)")
    ap.add_argument("--sysid", type=int, default=255, help="our (ground) system id")
    ap.add_argument("--target", type=int, default=1, help="aircraft system id (COMMS_SYSID)")
    ap.add_argument("--allow-arm", action="store_true", help="also send ARM/TAKEOFF/DISARM (bench, props off!)")
    ap.add_argument("--only", choices=["telemetry", "commands", "rates", "security"], help="run one step")
    ap.add_argument("--seconds", type=float, default=10, help="telemetry window")
    args = ap.parse_args()

    mav = load_goose_dialect()
    bench = Bench(args, mav)
    if not bench.wait_for_link():
        return 1
    steps = [args.only] if args.only else ["telemetry", "commands", "rates", "security"]
    for step in steps:
        if step == "telemetry":
            bench.telemetry(args.seconds)
        else:
            getattr(bench, step)()
    s = bench.conn.mav.signing
    print(f"\nsigning: good {s.goodsig_count} bad {s.badsig_count} unsigned {s.unsigned_count} rejected {s.reject_count}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
