#!/usr/bin/env python3
"""Decode datalog.bin written by the Teensy 4.1 logger. Everything is big-endian.

Packet: FEED | system | subsystem | level | timestamp_ms (u32) | len | payload | DEAD
        (bytes on disk: FE ED ... DE AD)
"""
import argparse
import csv
import struct
import sys

START = b"\xFE\xED"
END = b"\xDE\xAD"
HEADER = 10  # START(2), sys, sub, level, ts(4), len

LEVELS = {0x00: "INFO", 0x0F: "WARN", 0xFF: "ERROR"}

# (system, subsystem) -> (name, struct format, field names)
# ">" = big-endian; Teensy `int` is 32-bit. Add new sensors here.
DECODERS = {
    (0x00, 0x01): ("IMU", ">9f", ("accelX", "accelY", "accelZ", "gyroX", "gyroY", "gyroZ",
                                  "magX", "magY", "magZ")),
    (0x00, 0x02): ("BARO", ">2f", ("temp_C", "pressure_Pa")),
    (0x00, 0x04): ("PITOT", ">2f", ("pressure_Pa", "die_temp_C")),
    (0x00, 0x05): ("SENSOR_TRUST", ">4B", ("fail_threshold", "full_fail_threshold",
                                           "success_threshold_from_degraded",
                                           "success_threshold_from_failed")),
    (0x00, 0x03): ("GPS", ">2d5f4bi", ("latitude", "longitude", "altitude", "gr_speed",
                                       "n_vel", "e_vel", "d_vel",
                                       "utc_day", "utc_hour", "utc_min", "utc_sec", "utc_nanosec")),
}


def parse(data: bytes, allow_unknown: bool = False):
    """Yield (packet_dict, None) for good packets and (None, skipped_bytes) for resyncs."""
    i, n = 0, len(data)
    while i < n:
        if data[i:i + 2] != START:
            j = data.find(START, i + 1) if i + 1 < n else -1
            if j < 0:
                yield None, n - i
                return
            yield None, j - i
            i = j

        if i + HEADER + len(END) > n:  # too short for even an empty packet (truncated tail)
            yield None, n - i
            return

        system, sub, level = data[i + 2], data[i + 3], data[i + 4]
        (ts,) = struct.unpack_from(">I", data, i + 5)
        plen = data[i + 9]
        end = i + HEADER + plen

        valid = end + len(END) <= n and data[end:end + len(END)] == END
        if valid and not allow_unknown:
            # known sensor with the expected payload size, else treat as a false start
            entry = DECODERS.get((system, sub))
            valid = entry is not None and struct.calcsize(entry[1]) == plen
        if not valid:
            yield None, 1  # false start inside data, corruption, or truncation: skip 1 byte
            i += 1
            continue

        yield {
            "system": system, "sub": sub, "level": level, "ts": ts,
            "payload": data[i + HEADER:end],
        }, None
        i = end + len(END)


def decode_payload(pkt):
    entry = DECODERS.get((pkt["system"], pkt["sub"]))
    if not entry:
        return f"UNKNOWN_{pkt['system']:02X}_{pkt['sub']:02X}", {"raw": pkt["payload"].hex()}
    name, fmt, fields = entry
    if struct.calcsize(fmt) != len(pkt["payload"]):
        return name, {"raw": pkt["payload"].hex(), "error": "size mismatch"}
    return name, dict(zip(fields, struct.unpack(fmt, pkt["payload"])))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("file", help="datalog.bin")
    ap.add_argument("--csv", metavar="OUT", help="write CSV to OUT instead of printing")
    ap.add_argument("--allow-unknown", action="store_true",
                    help="accept packets from sensors not listed in DECODERS (raw hex output)")
    args = ap.parse_args()

    with open(args.file, "rb") as f:
        data = f.read()
    rows, good, skipped = [], 0, 0

    for pkt, skip in parse(data, args.allow_unknown):
        if pkt is None:
            skipped += skip
            continue
        good += 1
        name, fields = decode_payload(pkt)
        rows.append((pkt["ts"], name, LEVELS.get(pkt["level"], f"0x{pkt['level']:02X}"), fields))

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp_ms", "sensor", "level", "data"])
            for ts, name, lvl, fields in rows:
                w.writerow([ts, name, lvl, ";".join(f"{k}={v}" for k, v in fields.items())])
    else:
        for ts, name, lvl, fields in rows:
            print(f"{ts:>10} ms  {name:<6} {lvl:<5} {fields}")

    print(f"\n{good} packets decoded, {skipped} bytes skipped", file=sys.stderr)


if __name__ == "__main__":
    main()