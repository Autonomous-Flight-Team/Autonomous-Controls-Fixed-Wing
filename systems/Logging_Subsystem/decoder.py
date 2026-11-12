#!/usr/bin/env python3
"""Decode datalog.bin written by the Teensy 4.1 logger.

Packet: 0xFF | system | subsystem | level | timestamp_ms (u32 LE) | len | payload | 0x00
"""
import argparse
import csv
import struct
import sys

START, END = 0xFF, 0x00
HEADER = 8  # START, sys, sub, level, ts(4)

LEVELS = {0x00: "INFO", 0x0F: "WARN", 0xFF: "ERROR"}

# (system, subsystem) -> (name, struct format, field names)
# Teensy `int` is 32-bit, little-endian. Add new sensors here.
DECODERS = {
    (0x00, 0x01): ("IMU", "<3i", ("x", "y", "z")),
    (0x00, 0x02): ("BARO", "<i", ("z",)),
}


def parse(data: bytes, allow_unknown: bool = False):
    """Yield (packet_dict, None) for good packets and (None, skipped_bytes) for resyncs."""
    i, n = 0, len(data)
    while i < n:
        if data[i] != START:
            j = data.find(bytes([START]), i)
            if j < 0:
                yield None, n - i
                return
            yield None, j - i
            i = j

        if i + HEADER + 2 > n:  # not enough bytes left for even an empty packet
            yield None, n - i
            return

        system, sub, level = data[i + 1], data[i + 2], data[i + 3]
        (ts,) = struct.unpack_from("<I", data, i + 4)
        plen = data[i + 8]
        end = i + HEADER + 1 + plen

        valid = end < n and data[end] == END
        if valid and not allow_unknown:
            # known sensor with the expected payload size, else treat as a false start
            entry = DECODERS.get((system, sub))
            valid = entry is not None and struct.calcsize(entry[1]) == plen
        if not valid:
            # false start byte (0xFF inside data), corruption, or truncated tail: skip 1 byte
            yield None, 1
            i += 1
            continue

        yield {
            "system": system, "sub": sub, "level": level, "ts": ts,
            "payload": data[i + HEADER + 1:end],
        }, None
        i = end + 1


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

    data = open(args.file, "rb").read()
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