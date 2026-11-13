#!/usr/bin/env python3
"""Download the balancer's raw recording and turn it into CSV.

The firmware keeps the last `record_minutes` (default 10) of every raw
reading of both sensors, plus every tach pulse, in PSRAM. Fetch it over
Wi-Fi with:

    python tools/mpu_recording.py 192.168.111.113            # -> recording-<time>.csv
    python tools/mpu_recording.py 192.168.111.113 -o run.csv --bin run.bin
    python tools/mpu_recording.py --from-bin run.bin -o run.csv

Output CSV columns: t_s, source (A, B or TACH), x_g, y_g, z_g, axis (the axis
that sensor was measuring then: X, Y or Z). Times are the board's capture
timer, so readings and tach pulses line up to the microsecond.

File format (little endian): a 64-byte header
    "WBR1", header_len, record_size, count, sample_rate, range_g, capture_hz,
    sensors (uint32 each), axis_a, axis_b, filter_hz (uint8), padding,
then `count` 12-byte records: t_us (uint32, wraps after ~71 min), kind
(uint8: 0 tach, 1 sensor A, 2 sensor B), axis (uint8), x, y, z (int16 counts).
"""

import argparse
import math
import struct
import sys
import time
import urllib.request

HEADER = struct.Struct("<4s7I4B28x")
RECORD = struct.Struct("<IBB3h")
SOURCES = {0: "TACH", 1: "A", 2: "B"}


def load(data):
    magic, hlen, rsize, count, rate, range_g, cap_hz, sensors, ax_a, ax_b, filt, _ = HEADER.unpack_from(data, 0)
    if magic != b"WBR1":
        sys.exit("not a balancer recording (bad magic)")
    if rsize != RECORD.size:
        sys.exit(f"unexpected record size {rsize}")
    info = dict(count=count, rate=rate, range_g=range_g, sensors=sensors, axis_a="XYZ"[ax_a], axis_b="XYZ"[ax_b], filter=filt)
    g = range_g / 32768.0
    rows = []
    wraps = 0
    last = None
    for i in range(count):
        t_us, kind, axis, x, y, z = RECORD.unpack_from(data, hlen + i * rsize)
        if last is not None and t_us + 2**31 < last:  # uint32 wrapped
            wraps += 1
        last = t_us
        t = (t_us + wraps * 2**32) / 1e6
        rows.append((t, SOURCES.get(kind, "?"), x * g, y * g, z * g, "XYZ"[axis] if kind else ""))
    return info, rows


def summary(info, rows):
    if not rows:
        print("recording is empty")
        return
    span = rows[-1][0] - rows[0][0]
    print(f"{len(rows)} records over {span:.1f} s; {info['rate']} Hz, +/-{info['range_g']} g, filter {info['filter']} Hz")
    for src in ("A", "B"):
        r = [row for row in rows if row[1] == src]
        if not r:
            continue
        mean = [sum(v[k] for v in r) / len(r) for k in (2, 3, 4)]
        sd = [math.sqrt(max(0.0, sum(v[k] ** 2 for v in r) / len(r) - mean[k - 2] ** 2)) for k in (2, 3, 4)]
        print(f"  {src}: {len(r)} readings ({len(r) / span:.1f}/s), axis {r[-1][5]}; "
              f"mean x{mean[0]:+.3f} y{mean[1]:+.3f} z{mean[2]:+.3f} g; "
              f"sd x{sd[0]*1000:.1f} y{sd[1]*1000:.1f} z{sd[2]*1000:.1f} mg")
    tach = [row[0] for row in rows if row[1] == "TACH"]
    if len(tach) > 1:
        periods = [b - a for a, b in zip(tach, tach[1:]) if b > a]
        rpms = [60 / p for p in periods if p > 0]
        print(f"  tach: {len(tach)} pulses, {min(rpms):.0f}-{max(rpms):.0f} rpm")
    else:
        print(f"  tach: {len(tach)} pulses")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("host", nargs="?", help="the board's IP address or name")
    ap.add_argument("-o", "--out", help="CSV file (default recording-<time>.csv)")
    ap.add_argument("--bin", help="also save the raw download to this file")
    ap.add_argument("--from-bin", help="convert a saved .bin instead of downloading")
    args = ap.parse_args()
    if args.from_bin:
        with open(args.from_bin, "rb") as f:
            data = f.read()
    elif args.host:
        url = f"http://{args.host}/recording.bin"
        print(f"downloading {url} ...")
        t0 = time.time()
        with urllib.request.urlopen(url, timeout=60) as r:
            data = r.read()
        print(f"{len(data) / 1e6:.1f} MB in {time.time() - t0:.1f} s")
        if args.bin:
            with open(args.bin, "wb") as f:
                f.write(data)
    else:
        ap.error("give the board's address, or --from-bin")
    info, rows = load(data)
    summary(info, rows)
    out = args.out or time.strftime("recording-%Y%m%d-%H%M%S.csv")
    with open(out, "w", newline="") as f:
        f.write("t_s,source,x_g,y_g,z_g,axis\n")
        for t, src, x, y, z, ax in rows:
            f.write(f"{t:.6f},{src},{x:.5f},{y:.5f},{z:.5f},{ax}\n")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
