#!/usr/bin/env python3
"""GOOSE radio link logger with a live web dashboard (for range tests).

Listens on the ground RFD900x, sends signed 2 Hz ground heartbeats (so the aircraft
sees the link as UP), and every second records:
  - signal: RSSI and noise on both ends (RADIO_STATUS rssi/noise = this radio,
    remrssi/remnoise = the far radio), in raw SiK units and approximate dBm
  - packets: messages/s, bytes/s, and loss from gaps in the MAVLink sequence numbers
  - radio counters: rxerrors, fixed, txbuf; signing: bad signatures
  - markers you type in the dashboard ("corner of 45th")

Writes one CSV row per second and serves a dashboard at http://127.0.0.1:8765
(live charts, live packet feed, marker box). The page needs no internet.

    uv run --no-project --with pymavlink==2.4.50 --with pyserial \\
        tools/rf/link_logger.py --device /dev/cu.PL2303G-USBtoUART130 --key bench-test-only

On the aircraft side run air_sim.py with --seconds 0 (until Ctrl-C), or the Teensy.
Desk test without radios: --device tcpin:127.0.0.1:5761 here, and
air_sim.py --device tcp:127.0.0.1:5761 in another terminal.
"""
import argparse
import collections
import csv
import datetime
import hashlib
import json
import os
import signal
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, TOOLS)
from comms_bench import load_goose_dialect, parse_key  # noqa: E402  (also sets MAVLINK20)
from pymavlink import mavutil  # noqa: E402

RADIO_SRC = (ord("3"), ord("D"))  # SiK radio's own sysid/compid for RADIO_STATUS
LINK_TIMEOUT_S = 3.0
PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "link_dashboard.html")

CSV_FIELDS = ["time", "elapsed_s", "link_up", "msgs", "bytes", "lost", "loss_pct",
              "rssi", "remrssi", "noise", "remnoise", "rssi_dbm", "remrssi_dbm", "noise_dbm", "remnoise_dbm",
              "margin_db", "rem_margin_db", "rxerrors", "fixed", "txbuf", "badsig", "marker"]


def sik_dbm(raw):
    """SiK RSSI/noise units to approximate dBm (SiK docs: dBm ~= raw / 1.9 - 127)."""
    return None if raw is None else round(raw / 1.9 - 127, 1)


def brief(m):
    t = m.get_type()
    try:
        if t == "HEARTBEAT":
            return f"type {m.type} mode {m.custom_mode} state {m.system_status}"
        if t == "RADIO_STATUS":
            return f"rssi {m.rssi}/{m.remrssi} noise {m.noise}/{m.remnoise} txbuf {m.txbuf}% err {m.rxerrors} fix {m.fixed}"
        if t == "ATTITUDE":
            return f"roll {m.roll * 57.2958:+.1f} pitch {m.pitch * 57.2958:+.1f} yaw {m.yaw * 57.2958:+.1f}"
        if t == "GLOBAL_POSITION_INT":
            return f"lat {m.lat / 1e7:.5f} lon {m.lon / 1e7:.5f} alt {m.relative_alt / 1000:.1f} m"
        if t == "VFR_HUD":
            return f"airspd {m.airspeed:.1f} alt {m.alt:.1f} climb {m.climb:+.1f}"
        if t == "GOOSE_STATE":
            return f"system {m.system_state} flight {m.flight_state} faults 0x{m.active_faults:x}"
        if t == "STATUSTEXT":
            return m.text
        if t == "COMMAND_ACK":
            return f"cmd {m.command} result {m.result}"
    except AttributeError:
        pass
    return ""


class State:
    """Everything the dashboard shows. One lock; the MAVLink thread writes, HTTP threads read."""

    def __init__(self, args, csv_path):
        self.args = args
        self.lock = threading.Lock()
        self.t0 = time.time()
        self.seconds = []                           # one dict per second, whole run
        self.markers = []                           # {t, label}
        self.packets = collections.deque(maxlen=400)
        self.events = collections.deque(maxlen=20000)  # (id, kind, data) for SSE clients
        self.next_id = 1
        self.pending_markers = []
        self.csv_path = csv_path
        self.csv_file = open(csv_path, "w", newline="")
        self.csv = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.csv.writeheader()
        self.stop = False

    def elapsed(self):
        return time.time() - self.t0

    def _push(self, kind, data):
        self.events.append((self.next_id, kind, data))
        self.next_id += 1

    def add_packets(self, pkts):
        with self.lock:
            self.packets.extend(pkts)
            self._push("pkts", pkts)

    def add_marker(self, label):
        label = " ".join(label.split())[:120]
        if not label:
            return None
        mk = {"t": round(self.elapsed(), 1), "label": label}
        with self.lock:
            self.markers.append(mk)
            self.pending_markers.append(label)
            self._push("marker", mk)
        print(f"[{mk['t']:7.1f} s] MARKER: {label}", flush=True)
        return mk

    def add_second(self, row):
        with self.lock:
            row["marker"] = " | ".join(self.pending_markers)
            self.pending_markers.clear()
            self.seconds.append(row)
            self._push("sec", row)
            out = dict(row, time=datetime.datetime.fromtimestamp(self.t0 + row["elapsed_s"]).isoformat(timespec="seconds"))
            self.csv.writerow({k: ("" if out.get(k) is None else out.get(k)) for k in CSV_FIELDS})
            self.csv_file.flush()

    def snapshot(self):
        with self.lock:
            return {"seconds": list(self.seconds), "markers": list(self.markers), "packets": list(self.packets),
                    "last_id": self.next_id - 1, "device": self.args.device, "now": round(self.elapsed(), 3), "csv": os.path.basename(self.csv_path),
                    "started": datetime.datetime.fromtimestamp(self.t0).isoformat(timespec="seconds")}

    def events_after(self, last_id):
        with self.lock:
            return [e for e in self.events if e[0] > last_id]


def mav_loop(st, args, mav):
    conn = mavutil.mavlink_connection(args.device, baud=args.baud, source_system=255,
                                      source_component=mav.MAV_COMP_ID_MISSIONPLANNER)
    key = parse_key(args.key)
    conn.setup_signing(key, sign_outgoing=True,
                       allow_unsigned_callback=lambda _mav, msg_id: msg_id == mav.MAVLINK_MSG_ID_RADIO_STATUS)
    print(f"listening on {args.device} @ {args.baud}, key fingerprint {hashlib.sha256(key).hexdigest()[:12]}", flush=True)

    last_seq = {}
    last_hb = None
    next_hb = 0.0
    bucket = None
    sec_index = 0
    batch = []
    next_flush = 0.0
    last_badsig = 0
    last_print = 0.0

    def new_bucket():
        return {"msgs": 0, "bytes": 0, "lost": 0, "rs": None}

    bucket = new_bucket()
    while not st.stop:
        now = time.time()
        if now >= next_hb:
            try:
                conn.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, mav.MAV_STATE_ACTIVE)
            except OSError:
                pass  # tcpin with no client yet
            next_hb = now + 0.5

        if now - st.t0 >= sec_index + 1:  # close one-second bucket
            sec_index += 1
            rs = bucket["rs"]
            got = bucket["msgs"]
            lost = bucket["lost"]
            badsig = conn.mav.signing.badsig_count if conn.mav.signing else 0
            g = (lambda k: getattr(rs, k) if rs else None)
            row = {
                "elapsed_s": sec_index,
                "link_up": int(last_hb is not None and now - last_hb < LINK_TIMEOUT_S),
                "msgs": got, "bytes": bucket["bytes"], "lost": lost,
                "loss_pct": round(100 * lost / (got + lost), 1) if got + lost else None,
                "rssi": g("rssi"), "remrssi": g("remrssi"), "noise": g("noise"), "remnoise": g("remnoise"),
                "rssi_dbm": sik_dbm(g("rssi")), "remrssi_dbm": sik_dbm(g("remrssi")),
                "noise_dbm": sik_dbm(g("noise")), "remnoise_dbm": sik_dbm(g("remnoise")),
                "margin_db": round((rs.rssi - rs.noise) / 1.9, 1) if rs else None,
                "rem_margin_db": round((rs.remrssi - rs.remnoise) / 1.9, 1) if rs else None,
                "rxerrors": g("rxerrors"), "fixed": g("fixed"), "txbuf": g("txbuf"),
                "badsig": badsig - last_badsig,
            }
            last_badsig = badsig
            st.add_second(row)
            if now - last_print >= 5:
                last_print = now
                print(f"[{sec_index:5d} s] link {'UP  ' if row['link_up'] else 'LOST'} {got:3d} msg/s"
                      f" loss {row['loss_pct'] if row['loss_pct'] is not None else '-':>5}%"
                      f" rssi {row['rssi']}/{row['remrssi']} noise {row['noise']}/{row['remnoise']}", flush=True)
            bucket = new_bucket()

        if batch and now >= next_flush:
            st.add_packets(batch)
            batch = []
            next_flush = now + 0.2

        m = conn.recv_match(blocking=True, timeout=0.05)
        if m is None or m.get_type() == "BAD_DATA":
            continue
        t = m.get_type()
        src = (m.get_srcSystem(), m.get_srcComponent())
        seq = m.get_seq()
        gap = 0
        if src != RADIO_SRC:
            if src in last_seq:
                gap = (seq - last_seq[src] - 1) & 0xFF
            last_seq[src] = seq
        bucket["msgs"] += 1
        bucket["lost"] += gap
        try:
            bucket["bytes"] += len(m.get_msgbuf())
        except Exception:
            pass
        if t == "RADIO_STATUS" and src == RADIO_SRC:
            bucket["rs"] = m
        if t == "HEARTBEAT" and src[0] == args.target:
            last_hb = now
        batch.append({"t": round(now - st.t0, 2), "type": t, "sys": src[0], "comp": src[1], "seq": seq,
                      "gap": gap, "info": brief(m)})


class Handler(BaseHTTPRequestHandler):
    st = None

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            with open(PAGE, "rb") as f:
                self._send(200, f.read(), "text/html; charset=utf-8")
        elif self.path == "/csv":
            with open(self.st.csv_path, "rb") as f:
                self.send_response(200)
                body = f.read()
                self.send_header("Content-Type", "text/csv")
                self.send_header("Content-Disposition", f'attachment; filename="{os.path.basename(self.st.csv_path)}"')
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                snap = self.st.snapshot()
                last = snap["last_id"]
                self.wfile.write(f"event: hello\ndata: {json.dumps(snap)}\n\n".encode())
                self.wfile.flush()
                while not self.st.stop:
                    time.sleep(0.25)
                    evs = self.st.events_after(last)
                    if evs:
                        last = evs[-1][0]
                        out = "".join(f"event: {k}\ndata: {json.dumps(d)}\n\n" for _, k, d in evs)
                        self.wfile.write(out.encode())
                    else:
                        self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self._send(404, b"not found", "text/plain")

    def do_POST(self):
        if self.path != "/marker":
            return self._send(404, b"not found", "text/plain")
        n = int(self.headers.get("Content-Length", 0) or 0)
        try:
            label = json.loads(self.rfile.read(min(n, 4096)) or b"{}").get("label", "")
        except ValueError:
            return self._send(400, b"bad json", "text/plain")
        mk = self.st.add_marker(str(label))
        self._send(200 if mk else 400, json.dumps(mk).encode(), "application/json")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", required=True, help="ground radio serial port, or tcpin:127.0.0.1:5761 for a desk test")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--key", required=True, help="signing key: 64 hex chars, or a passphrase (SHA-256)")
    ap.add_argument("--target", type=int, default=1, help="aircraft system id")
    ap.add_argument("--port", type=int, default=8765, help="dashboard port")
    ap.add_argument("--host", default="127.0.0.1",
                    help="dashboard address; 0.0.0.0 lets a phone on the same Wi-Fi/hotspot open it (no password!)")
    ap.add_argument("--out", default="range_logs", help="folder for the CSV log")
    ap.add_argument("--no-browser", action="store_true", help="don't open the dashboard automatically")
    args = ap.parse_args()

    mav = load_goose_dialect()
    os.makedirs(args.out, exist_ok=True)
    csv_path = os.path.join(args.out, datetime.datetime.now().strftime("range_%Y%m%d_%H%M%S.csv"))
    st = State(args, csv_path)
    Handler.st = st

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{args.port}/"
    print(f"dashboard: {url}" + (f"  (also on this machine's network address, port {args.port})" if args.host == "0.0.0.0" else ""))
    print(f"logging to {csv_path}", flush=True)
    if not args.no_browser:
        webbrowser.open(url)

    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)   # also when started from a shell that ignores SIGINT
    signal.signal(signal.SIGTERM, interrupt)

    t = threading.Thread(target=mav_loop, args=(st, args, mav), daemon=True)
    t.start()
    try:
        while t.is_alive():
            t.join(0.5)
    except KeyboardInterrupt:
        pass
    signal.signal(signal.SIGINT, signal.SIG_IGN)   # a second Ctrl-C (or uv forwarding it) must not cut the summary
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    st.stop = True
    secs = st.seconds
    up = sum(r["link_up"] for r in secs)
    got = sum(r["msgs"] for r in secs)
    lost = sum(r["lost"] for r in secs)
    print(f"\n{len(secs)} s logged, link up {up} s, {got} msgs, {lost} lost"
          f" ({100 * lost / (got + lost):.1f}%)" if got + lost else f"\n{len(secs)} s logged, no messages")
    print(f"CSV: {csv_path}")
    st.csv_file.close()


if __name__ == "__main__":
    main()
