#!/usr/bin/env python3
"""GOOSE aircraft simulator: a Python stand-in for the Teensy comms firmware.

Plays the aircraft side of the link so the ground station (tools/comms_bench.py) can be
tested without the Teensy: signed MAVLink v2 heartbeats, GOOSE_STATE, telemetry streams
at the comms_config.h default rates, the stub command handler's guards, command
de-duplication, SET/GET_MESSAGE_INTERVAL, GOOSE_SET_LOG_RATE, a byte budget and the
RADIO_STATUS txbuf slow-down. Behaviour follows systems/Comms_Subsystem/comms.cpp,
command_handler.cpp and telemetry_source.cpp (dummy telemetry, stub handler).

Over the network (no radio) — the aircraft listens, the ground connects:

    uv run --no-project --with pymavlink==2.4.50 --with pyserial tools/air_sim.py \\
        --device tcpin:0.0.0.0:5760 --key bench-test-only
    uv run --no-project --with pymavlink==2.4.50 --with pyserial tools/comms_bench.py \\
        --device tcp:<aircraft ip>:5760 --key bench-test-only

Over an RFD900x:  --device COM5 (Windows) or /dev/tty.usbserial-XXXX, baud 57600.

The key rules are the same as comms_bench.py: 64 hex chars, or a passphrase (SHA-256).
"""
import argparse
import hashlib
import math
import os
import sys
import time

os.environ["MAVLINK20"] = "1"  # must be set before importing mavutil

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from comms_bench import load_goose_dialect, parse_key  # noqa: E402
from pymavlink import mavutil  # noqa: E402

# ---- comms_config.h
SYSID, COMPID = 1, 1
HEARTBEAT_US = 1_000_000
GCS_TIMEOUT_S = 3.0
TX_BYTES_PER_SEC = 3000
MIN_INTERVAL_US = 20_000
CMD_DUP_WINDOW_S = 3.0
RADIO_STATUS_STALE_S = 5.0
LOG_DEFAULT_US = 1_000_000
MAX_LOG_STREAMS = 8
FORCE_ARM_MAGIC = 21196.0
RADIO_SYSID, RADIO_COMPID = ord("3"), ord("D")


class Sim:
    """Dummy telemetry + stub command handler (telemetry_source.cpp, command_handler.cpp)."""

    def __init__(self, m):
        self.m = m
        self.system = m.GOOSE_SYSTEM_STATE_STANDBY
        self.flight = m.GOOSE_FLIGHT_STATE_ON_GROUND
        self.faults = 0
        self.flags = 0
        self.critical_entries = 0
        self.sensors = [m.GOOSE_SENSOR_STATUS_NOMINAL] * 7 + [m.GOOSE_SENSOR_STATUS_UNINITIALIZED]
        self.t0 = time.monotonic()
        self._last_sys, self._sys_since = None, 0.0
        self._last_fl, self._fl_since = None, 0.0

    def ms(self):
        return int((time.monotonic() - self.t0) * 1000)

    def armed(self):
        m = self.m
        return self.system in (m.GOOSE_SYSTEM_STATE_ACTIVE, m.GOOSE_SYSTEM_STATE_DEGRADED,
                               m.GOOSE_SYSTEM_STATE_CRITICAL, m.GOOSE_SYSTEM_STATE_EMERGENCY)

    def update(self):
        m, now = self.m, self.ms()
        if self.system != self._last_sys:
            self._last_sys, self._sys_since = self.system, now
        if self.system == m.GOOSE_SYSTEM_STATE_PREFLIGHT and now - self._sys_since >= 1000:
            self.system = m.GOOSE_SYSTEM_STATE_STANDBY
        if self.flight != self._last_fl:
            self._last_fl, self._fl_since = self.flight, now
        if now - self._fl_since < 5000:
            return
        step = {m.GOOSE_FLIGHT_STATE_TAKEOFF_ROLL: m.GOOSE_FLIGHT_STATE_CLIMB,
                m.GOOSE_FLIGHT_STATE_CLIMB: m.GOOSE_FLIGHT_STATE_CRUISE,
                m.GOOSE_FLIGHT_STATE_RTL: m.GOOSE_FLIGHT_STATE_DESCENT,
                m.GOOSE_FLIGHT_STATE_DESCENT: m.GOOSE_FLIGHT_STATE_LANDING,
                m.GOOSE_FLIGHT_STATE_LANDING: m.GOOSE_FLIGHT_STATE_LANDED}
        self.flight = step.get(self.flight, self.flight)

    def snapshot(self):
        flags = (self.flags & ~self.m.GOOSE_STATE_FLAGS_ARMED) | (self.m.GOOSE_STATE_FLAGS_ARMED if self.armed() else 0)
        return (self.faults, self.system, self.flight, self.critical_entries, flags, tuple(self.sensors))

    # -- stub command handler
    def arm(self, arm):
        m = self.m
        if arm:
            if self.armed():
                return m.MAV_RESULT_ACCEPTED
            if self.system != m.GOOSE_SYSTEM_STATE_STANDBY:
                return m.MAV_RESULT_DENIED
            self.system = m.GOOSE_SYSTEM_STATE_ACTIVE
            return m.MAV_RESULT_ACCEPTED
        if not self.armed():
            return m.MAV_RESULT_ACCEPTED
        if self.flight not in (m.GOOSE_FLIGHT_STATE_ON_GROUND, m.GOOSE_FLIGHT_STATE_TAKEOFF_ROLL,
                               m.GOOSE_FLIGHT_STATE_REJECTED_TAKEOFF, m.GOOSE_FLIGHT_STATE_LANDED):
            return m.MAV_RESULT_DENIED  # never disarms in the air, even forced
        self.system = m.GOOSE_SYSTEM_STATE_FAULT_GROUND if self.faults else m.GOOSE_SYSTEM_STATE_STANDBY
        self.flight = m.GOOSE_FLIGHT_STATE_ON_GROUND
        return m.MAV_RESULT_ACCEPTED

    def takeoff(self):
        m = self.m
        if self.system != m.GOOSE_SYSTEM_STATE_ACTIVE or self.flight != m.GOOSE_FLIGHT_STATE_ON_GROUND:
            return m.MAV_RESULT_DENIED
        self.flight = m.GOOSE_FLIGHT_STATE_TAKEOFF_ROLL
        return m.MAV_RESULT_ACCEPTED

    def rtl(self):
        m = self.m
        if self.flight == m.GOOSE_FLIGHT_STATE_RTL:
            return m.MAV_RESULT_ACCEPTED
        if self.flight not in (m.GOOSE_FLIGHT_STATE_CLIMB, m.GOOSE_FLIGHT_STATE_CRUISE, m.GOOSE_FLIGHT_STATE_LOITER):
            return m.MAV_RESULT_DENIED
        self.flight = m.GOOSE_FLIGHT_STATE_RTL
        return m.MAV_RESULT_ACCEPTED

    def loiter(self):
        m = self.m
        if self.flight == m.GOOSE_FLIGHT_STATE_LOITER:
            return m.MAV_RESULT_ACCEPTED
        if self.flight != m.GOOSE_FLIGHT_STATE_CRUISE:
            return m.MAV_RESULT_DENIED
        self.flight = m.GOOSE_FLIGHT_STATE_LOITER
        return m.MAV_RESULT_ACCEPTED

    def resume(self):
        m = self.m
        if self.flight not in (m.GOOSE_FLIGHT_STATE_LOITER, m.GOOSE_FLIGHT_STATE_RTL):
            return m.MAV_RESULT_DENIED
        self.flight = m.GOOSE_FLIGHT_STATE_CRUISE
        return m.MAV_RESULT_ACCEPTED

    def fault_clear(self):
        m = self.m
        if self.system not in (m.GOOSE_SYSTEM_STATE_FAULT_GROUND, m.GOOSE_SYSTEM_STATE_IN_AIR_RESET):
            return m.MAV_RESULT_DENIED
        self.system = m.GOOSE_SYSTEM_STATE_PREFLIGHT
        return m.MAV_RESULT_ACCEPTED

    def bench(self, enter):
        m = self.m
        if enter:
            if self.system != m.GOOSE_SYSTEM_STATE_STANDBY:
                return m.MAV_RESULT_ACCEPTED if self.system == m.GOOSE_SYSTEM_STATE_BENCH else m.MAV_RESULT_DENIED
            self.system = m.GOOSE_SYSTEM_STATE_BENCH
            return m.MAV_RESULT_ACCEPTED
        if self.system != m.GOOSE_SYSTEM_STATE_BENCH:
            return m.MAV_RESULT_DENIED
        self.system = m.GOOSE_SYSTEM_STATE_PREFLIGHT
        return m.MAV_RESULT_ACCEPTED


class Row:
    def __init__(self, default_us, msg_id=None, log_key=None):
        self.default_us = default_us
        self.interval_us = default_us
        self.msg_id = msg_id
        self.log_key = log_key  # (system, subsystem) for GOOSE_LOG streams
        self.next_due = 0.0
        self.sample = None      # latest log record (ts_ms, level, payload)
        self.dropped = 0


class Aircraft:
    def __init__(self, args, m):
        self.m = m
        self.args = args
        self.conn = mavutil.mavlink_connection(args.device, baud=args.baud, source_system=SYSID,
                                               source_component=COMPID)
        self.key = parse_key(args.key)
        self.conn.setup_signing(self.key, sign_outgoing=True,
                                allow_unsigned_callback=lambda _mav, msg_id: msg_id == m.MAVLINK_MSG_ID_RADIO_STATUS)
        self.sim = Sim(m)
        self.rows = [Row(us, msg_id=mid) for mid, us in (
            (m.MAVLINK_MSG_ID_GOOSE_STATE, 500_000),
            (m.MAVLINK_MSG_ID_SYS_STATUS, 1_000_000),
            (m.MAVLINK_MSG_ID_ATTITUDE, 100_000),
            (m.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 250_000),
            (m.MAVLINK_MSG_ID_GPS_RAW_INT, 500_000),
            (m.MAVLINK_MSG_ID_VFR_HUD, 250_000),
            (m.MAVLINK_MSG_ID_SCALED_IMU, 0),
            (m.MAVLINK_MSG_ID_SCALED_PRESSURE, 1_000_000),
            (m.MAVLINK_MSG_ID_BATTERY_STATUS, 1_000_000))]
        self.log_interval_request = 0
        self.acks = []       # (command, result, sys, comp), max 4
        self.events = []     # callables that send one frame, max 8
        self.cmd_cache = []  # (sysid, compid, command, params, result, time)
        self.next_hb = 0.0
        self.last_state = None
        self.last_gcs = None
        self.txbuf, self.txbuf_time = 100, None
        self.tokens, self.last_refill = TX_BYTES_PER_SEC / 4, time.monotonic()
        self.stats = dict(rx=0, tx=0, tx_bytes=0, cmds=0, dups=0, dropped=0)
        self.next_fake_log = 0.0
        print(f"aircraft sim on {args.device}, key fingerprint {hashlib.sha256(self.key).hexdigest()[:12]}")

    # ---------------------------------------------------------------- helpers
    def find(self, msg_id):
        return next((r for r in self.rows if r.msg_id == msg_id), None)

    def find_log(self, key):
        return next((r for r in self.rows if r.log_key == key), None)

    def find_or_add_log(self, key):
        r = self.find_log(key)
        if r is None and sum(1 for x in self.rows if x.log_key) < MAX_LOG_STREAMS:
            r = Row(LOG_DEFAULT_US, log_key=key)
            if self.log_interval_request:
                self.set_interval(r, self.log_interval_request)
            self.rows.append(r)
        return r

    def set_interval(self, r, requested_us):
        if requested_us < 0:
            r.interval_us = 0
        elif requested_us == 0:
            r.interval_us = r.default_us
        else:
            r.interval_us = max(MIN_INTERVAL_US, min(int(requested_us), 2**32 - 1))
        r.next_due = time.monotonic()

    def rate_scale(self):
        if self.txbuf_time is None or time.monotonic() - self.txbuf_time > RADIO_STATUS_STALE_S:
            return 1
        return 4 if self.txbuf < 25 else 2 if self.txbuf < 50 else 1

    def frame_bound(self, msg_id):
        return 12 + self.m.mavlink_map[msg_id].unpacker.size + 13

    def budget_ok(self, msg_id):
        return self.tokens >= self.frame_bound(msg_id)

    def sent(self, msg_id):
        n = self.frame_bound(msg_id)  # upper bound; MAVLink 2 trims trailing zeros
        self.tokens -= n
        self.stats["tx"] += 1
        self.stats["tx_bytes"] += n

    def gcs_ok(self):
        return self.last_gcs is not None and time.monotonic() - self.last_gcs <= GCS_TIMEOUT_S

    # ---------------------------------------------------------------- receive
    def receive(self):
        m = self.m
        for _ in range(64):
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                return
            t = msg.get_type()
            if t == "BAD_DATA":
                continue
            self.stats["rx"] += 1
            if t == "HEARTBEAT" and msg.type == m.MAV_TYPE_GCS:
                if self.last_gcs is None or not self.gcs_ok():
                    print(f"[{self.sim.ms() / 1000:7.1f}s] ground station link UP (sysid {msg.get_srcSystem()})")
                self.last_gcs = time.monotonic()
            elif t == "RADIO_STATUS" and (msg.get_srcSystem(), msg.get_srcComponent()) == (RADIO_SYSID, RADIO_COMPID):
                self.txbuf, self.txbuf_time = msg.txbuf, time.monotonic()
            elif t == "COMMAND_LONG":
                self.command_long(msg)
            elif t == "COMMAND_INT" and self.targets_us(msg.target_system, msg.target_component):
                self.push_ack(msg.command, m.MAV_RESULT_COMMAND_LONG_ONLY, msg)

    def targets_us(self, sysid, compid):
        return sysid in (SYSID, 0) and compid in (COMPID, 0)

    def push_ack(self, command, result, msg):
        if len(self.acks) == 4:
            self.acks.pop(0)
        self.acks.append((command, result, msg.get_srcSystem(), msg.get_srcComponent()))

    def push_event(self, fn):
        if len(self.events) == 8:
            self.events.pop(0)
        self.events.append(fn)

    def command_long(self, c):
        m = self.m
        if not self.targets_us(c.target_system, c.target_component):
            return
        self.stats["cmds"] += 1
        now = time.monotonic()
        params = (c.param1, c.param2, c.param3, c.param4, c.param5, c.param6, c.param7)
        src = (c.get_srcSystem(), c.get_srcComponent())
        prev = next((e for e in self.cmd_cache if e[:3] == (*src, c.command) and e[3] == params
                     and now - e[5] <= CMD_DUP_WINDOW_S), None)
        if c.confirmation > 0 and prev:
            self.stats["dups"] += 1
            self.push_ack(c.command, prev[4], c)
            return
        result = self.execute(c, params)
        self.cmd_cache = (self.cmd_cache + [(*src, c.command, params, result, now)])[-4:]
        name = m.enums["MAV_CMD"][c.command].name if c.command in m.enums["MAV_CMD"] else c.command
        print(f"[{self.sim.ms() / 1000:7.1f}s] {name} -> {m.enums['MAV_RESULT'][result].name}")
        self.push_ack(c.command, result, c)

    def execute(self, c, p):
        m, s = self.m, self.sim
        cmd = c.command
        if cmd == m.MAV_CMD_COMPONENT_ARM_DISARM:
            return s.arm(p[0] == 1) if p[0] in (0, 1) else m.MAV_RESULT_DENIED
        if cmd == m.MAV_CMD_NAV_TAKEOFF:
            return s.takeoff()
        if cmd == m.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            return s.rtl()
        if cmd == m.MAV_CMD_NAV_LOITER_UNLIM:
            return s.loiter()
        if cmd == m.MAV_CMD_GOOSE_RESUME_MISSION:
            return s.resume()
        if cmd == m.MAV_CMD_GOOSE_FAULT_CLEAR:
            return s.fault_clear()
        if cmd == m.MAV_CMD_GOOSE_BENCH:
            return s.bench(p[0] == 1) if p[0] in (0, 1) else m.MAV_RESULT_DENIED
        if cmd == m.MAV_CMD_SET_MESSAGE_INTERVAL:
            mid, interval = int(p[0]), int(p[1])
            if mid == m.MAVLINK_MSG_ID_GOOSE_LOG:
                self.log_interval_request = interval
                for r in self.rows:
                    if r.log_key:
                        self.set_interval(r, interval)
                return m.MAV_RESULT_ACCEPTED
            r = self.find(mid)
            if r is None:
                return m.MAV_RESULT_DENIED
            self.set_interval(r, interval)
            return m.MAV_RESULT_ACCEPTED
        if cmd == m.MAV_CMD_GET_MESSAGE_INTERVAL:
            mid = int(p[0])
            self.push_event(lambda: self.conn.mav.message_interval_send(mid, self.interval_for_report(mid)))
            return m.MAV_RESULT_ACCEPTED
        if cmd == m.MAV_CMD_GOOSE_SET_LOG_RATE:
            if not (0 <= p[0] <= 255 and 0 <= p[1] <= 255):
                return m.MAV_RESULT_DENIED
            r = self.find_or_add_log((int(p[0]), int(p[1])))
            if r is None:
                return m.MAV_RESULT_DENIED
            self.set_interval(r, int(p[2]))
            return m.MAV_RESULT_ACCEPTED
        return m.MAV_RESULT_UNSUPPORTED

    def interval_for_report(self, mid):
        m = self.m
        if mid == m.MAVLINK_MSG_ID_HEARTBEAT:
            return HEARTBEAT_US
        if mid == m.MAVLINK_MSG_ID_GOOSE_LOG:
            req = self.log_interval_request
            return -1 if req < 0 else LOG_DEFAULT_US if req == 0 else req
        r = self.find(mid)
        if r is None:
            return 0
        return -1 if r.interval_us == 0 else r.interval_us

    # ---------------------------------------------------------------- transmit
    def send_heartbeat(self):
        m, s = self.m, self.sim
        base = m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (m.MAV_MODE_FLAG_SAFETY_ARMED if s.armed() else 0)
        mav_state = {m.GOOSE_SYSTEM_STATE_BOOT: m.MAV_STATE_BOOT,
                     m.GOOSE_SYSTEM_STATE_PREFLIGHT: m.MAV_STATE_CALIBRATING,
                     m.GOOSE_SYSTEM_STATE_STANDBY: m.MAV_STATE_STANDBY,
                     m.GOOSE_SYSTEM_STATE_FAULT_GROUND: m.MAV_STATE_STANDBY,
                     m.GOOSE_SYSTEM_STATE_BENCH: m.MAV_STATE_STANDBY,
                     m.GOOSE_SYSTEM_STATE_ACTIVE: m.MAV_STATE_ACTIVE,
                     m.GOOSE_SYSTEM_STATE_DEGRADED: m.MAV_STATE_ACTIVE,
                     m.GOOSE_SYSTEM_STATE_CRITICAL: m.MAV_STATE_CRITICAL,
                     m.GOOSE_SYSTEM_STATE_EMERGENCY: m.MAV_STATE_EMERGENCY,
                     m.GOOSE_SYSTEM_STATE_IN_AIR_RESET: m.MAV_STATE_CRITICAL}.get(s.system, m.MAV_STATE_UNINIT)
        self.conn.mav.heartbeat_send(m.MAV_TYPE_FIXED_WING, m.MAV_AUTOPILOT_GENERIC, base,
                                     s.flight | (s.system << 8), mav_state)
        self.sent(m.MAVLINK_MSG_ID_HEARTBEAT)

    def send_goose_state(self):
        faults, system, flight, crit, flags, sensors = snap = self.sim.snapshot()
        self.conn.mav.goose_state_send(self.sim.ms(), faults, system, flight, crit, flags, list(sensors))
        self.sent(self.m.MAVLINK_MSG_ID_GOOSE_STATE)
        self.last_state = snap

    def send_stream(self, r):
        m, mav, t = self.m, self.conn.mav, self.sim.ms()
        mid = m.MAVLINK_MSG_ID_GOOSE_LOG if r.log_key else r.msg_id
        if not self.budget_ok(mid):
            return False
        ph10 = 2 * math.pi * (t % 10000) / 10000
        ph60 = 2 * math.pi * (t % 60000) / 60000
        lat = 476530000 + int(5000 * math.sin(ph60))  # around Seattle, like the dummy source
        lon = -1223050000 + int(5000 * math.cos(ph60))
        hdg = int(ph60 * 18000 / math.pi) % 36000
        if r.log_key:
            ts, level, payload = r.sample
            mav.goose_log_send(ts, r.log_key[0], r.log_key[1], level, len(payload),
                               list(payload) + [0] * (64 - len(payload)))
        elif mid == m.MAVLINK_MSG_ID_GOOSE_STATE:
            self.send_goose_state()
            return True
        elif mid == m.MAVLINK_MSG_ID_SYS_STATUS:
            bits = [m.MAV_SYS_STATUS_SENSOR_3D_GYRO | m.MAV_SYS_STATUS_SENSOR_3D_ACCEL,
                    m.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE, m.MAV_SYS_STATUS_SENSOR_GPS,
                    m.MAV_SYS_STATUS_SENSOR_3D_MAG, m.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
                    m.MAV_SYS_STATUS_SENSOR_BATTERY, m.MAV_SYS_STATUS_SENSOR_RC_RECEIVER, 0]
            present = health = 0
            for b, st in zip(bits, self.sim.sensors):
                if st != m.GOOSE_SENSOR_STATUS_UNINITIALIZED:
                    present |= b
                if st in (m.GOOSE_SENSOR_STATUS_NOMINAL, m.GOOSE_SENSOR_STATUS_DEGRADED):
                    health |= b
            sig = self.conn.mav.signing
            errs = min(65535, sig.badsig_count)
            mav.sys_status_send(present, present, health, 0, 12400, 850, 80, 0, errs, 0, 0, 0, 0)
        elif mid == m.MAVLINK_MSG_ID_ATTITUDE:
            mav.attitude_send(t, 0.3 * math.sin(ph10), 0.1 * math.cos(ph10), ph10 - math.pi,
                              0.3 * math.cos(ph10), -0.1 * math.sin(ph10), 0.6)
        elif mid == m.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            mav.global_position_int_send(t, lat, lon, 120000, 100000, int(1500 * math.cos(ph60)),
                                         int(-1500 * math.sin(ph60)), 0, hdg)
        elif mid == m.MAVLINK_MSG_ID_GPS_RAW_INT:
            mav.gps_raw_int_send(t * 1000, m.GPS_FIX_TYPE_3D_FIX, lat, lon, 120000, 90, 140, 1500, hdg, 14)
        elif mid == m.MAVLINK_MSG_ID_VFR_HUD:
            mav.vfr_hud_send(16.0, 15.0, (t // 1000) % 360, 55, 120.0, 0.0)
        elif mid == m.MAVLINK_MSG_ID_SCALED_IMU:
            mav.scaled_imu_send(t, 0, 0, -1000, 0, 0, 0, 200, 0, 400)
        elif mid == m.MAVLINK_MSG_ID_SCALED_PRESSURE:
            mav.scaled_pressure_send(t, 1013.25, 1.6, 2500)
        elif mid == m.MAVLINK_MSG_ID_BATTERY_STATUS:
            mav.battery_status_send(0, m.MAV_BATTERY_FUNCTION_ALL, m.MAV_BATTERY_TYPE_LIPO, 32767,
                                    [12400] + [65535] * 9, 850, t // 4000, -1, 80)
        else:
            return False
        self.sent(mid)
        return True

    def transmit(self):
        m, now = self.m, time.monotonic()
        while self.acks:  # 1. acks
            cmd, res, sysid, compid = self.acks.pop(0)
            self.conn.mav.command_ack_send(cmd, res, 0, 0, sysid, compid)
            self.sent(m.MAVLINK_MSG_ID_COMMAND_ACK)
        if now >= self.next_hb:  # 2. heartbeat
            self.send_heartbeat()
            self.next_hb = max(self.next_hb + HEARTBEAT_US / 1e6, now) if self.next_hb else now + 1
        gs = self.find(m.MAVLINK_MSG_ID_GOOSE_STATE)  # 3. GOOSE_STATE on change or when due
        gs_due = gs.interval_us > 0 and now >= gs.next_due
        if self.sim.snapshot() != self.last_state or gs_due:
            self.send_goose_state()
            if gs_due:
                gs.next_due = now + gs.interval_us * self.rate_scale() / 1e6
        while self.events:  # 4. events
            self.events.pop(0)()
            self.stats["tx"] += 1
        blocked = False  # 5. rate streams, most overdue first
        while not blocked:
            due = [r for r in self.rows if r.interval_us and r.next_due <= now and (r.log_key is None or r.sample)
                   and r is not gs]
            if not due:
                break
            r = min(due, key=lambda x: x.next_due)
            if self.send_stream(r):
                r.next_due = max(r.next_due + r.interval_us * self.rate_scale() / 1e6, now)
            else:
                blocked = True
        if blocked:  # back-pressure: skip this period, no backlog
            for r in self.rows:
                if r.interval_us and r.next_due <= now:
                    r.dropped += 1
                    self.stats["dropped"] += 1
                    r.next_due = now + r.interval_us * self.rate_scale() / 1e6

    def refill(self):
        now = time.monotonic()
        self.tokens = min(TX_BYTES_PER_SEC / 4, self.tokens + (now - self.last_refill) * TX_BYTES_PER_SEC)
        self.last_refill = now

    # ---------------------------------------------------------------- main loop
    def run(self, duration):
        end = time.monotonic() + duration if duration else None
        next_status = time.monotonic() + 5
        gcs_was_ok = False
        while end is None or time.monotonic() < end:
            now = time.monotonic()
            self.sim.update()
            if now >= self.next_fake_log:  # fake 10 Hz IMU log records, as src/main.cpp does
                self.next_fake_log = now + 0.1
                t = self.sim.ms()
                payload = (t % 1000).to_bytes(4, "little", signed=True) + (0).to_bytes(4, "little") + \
                    (-1000).to_bytes(4, "little", signed=True)
                r = self.find_or_add_log((0x00, 0x01))
                r.sample = (t, 0x00, payload)
            self.receive()
            self.refill()
            self.transmit()
            if gcs_was_ok and not self.gcs_ok():
                print(f"[{self.sim.ms() / 1000:7.1f}s] ground station link LOST")
            gcs_was_ok = self.gcs_ok()
            if now >= next_status:
                next_status = now + 5
                s, sig = self.stats, self.conn.mav.signing
                print(f"[{self.sim.ms() / 1000:7.1f}s] link {'OK' if self.gcs_ok() else 'LOST'} rx {s['rx']} "
                      f"tx {s['tx']} (~{s['tx_bytes']} B) cmds {s['cmds']} dups {s['dups']} dropped {s['dropped']} "
                      f"badsig {sig.badsig_count} unsigned {sig.unsigned_count} txbuf {self.txbuf}%")
            time.sleep(0.005)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", required=True,
                    help="tcpin:0.0.0.0:5760 (listen), tcp:host:port, COM5, /dev/tty.usbserial-XXXX ...")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--key", required=True, help="signing key: 64 hex chars, or a passphrase (SHA-256)")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = run until Ctrl-C)")
    args = ap.parse_args()
    mav = load_goose_dialect()
    try:
        Aircraft(args, mav).run(args.seconds)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
