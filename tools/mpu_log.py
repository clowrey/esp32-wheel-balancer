#!/usr/bin/env python3
"""Record the balancer's raw MPU-6050 stream from USB to a CSV file.

Turn the stream on with "PC log" on the LIVE screen (or `pc_log: true` in the
YAML), then run:

    python tools/mpu_log.py COM7                 # Windows
    python tools/mpu_log.py /dev/ttyACM0 -o run1.csv -t 60

Every reading of both sensors goes into the CSV, all three axes, in g, with
the time it was taken (from the board's capture timer, in seconds). Every few
seconds it prints each sensor's reading rate, mean, noise (standard
deviation) and the magnitude of gravity, which should be close to 1.00 g.

The firmware sends lines tagged mpu.raw:
    MPU CFG range=2 rate=250 ...
    MPU A <t0_us> <n> dt_us,x,y,z dt_us,x,y,z ...     (raw counts)
Needs pyserial (`pip install pyserial`). Opening the port does not reset the
board.
"""

import argparse
import math
import re
import sys
import time

import serial

LINE_RE = re.compile(r"MPU ([AB]) (\d+) (\d+) (.*)")
CFG_RE = re.compile(r"MPU CFG (.*)")
ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


class Stats:
    def __init__(self):
        self.reset()

    def reset(self):
        self.n = 0
        self.s = [0.0] * 3
        self.q = [0.0] * 3
        self.t_first = None
        self.t_last = None
        self.gaps = 0

    def add(self, t, g, period):
        if self.t_last is not None and period and t - self.t_last > 1.5 * period:
            self.gaps += 1
        if self.t_first is None:
            self.t_first = t
        self.t_last = t
        self.n += 1
        for k in range(3):
            self.s[k] += g[k]
            self.q[k] += g[k] * g[k]

    def line(self, name):
        if self.n < 2:
            return f"{name}: no readings"
        mean = [v / self.n for v in self.s]
        sd = [math.sqrt(max(0.0, self.q[k] / self.n - mean[k] ** 2)) for k in range(3)]
        span = self.t_last - self.t_first
        rate = (self.n - 1) / span if span > 0 else 0
        mag = math.sqrt(sum(m * m for m in mean))
        return (
            f"{name}: {rate:6.1f}/s  mean x{mean[0]:+.3f} y{mean[1]:+.3f} z{mean[2]:+.3f} g  |g| {mag:.3f}"
            f"  noise x{sd[0] * 1000:.2f} y{sd[1] * 1000:.2f} z{sd[2] * 1000:.2f} mg  gaps {self.gaps}"
        )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="serial port, e.g. COM7 or /dev/ttyACM0")
    ap.add_argument("-o", "--out", default=None, help="CSV file (default mpu-<time>.csv)")
    ap.add_argument("-t", "--seconds", type=float, default=0, help="stop after this many seconds (default: Ctrl+C)")
    ap.add_argument("-r", "--range", type=int, default=0, help="+/- g range if no MPU CFG line is seen (default 2)")
    ap.add_argument("--every", type=float, default=5, help="seconds between summaries (default 5)")
    args = ap.parse_args()

    out = args.out or time.strftime("mpu-%Y%m%d-%H%M%S.csv")
    g_per_count = (args.range or 2) / 32768.0
    period = None
    port = serial.Serial()
    port.port = args.port
    port.baudrate = 115200
    port.timeout = 0.2
    port.dtr = False  # don't reset the board on open
    port.rts = False
    port.open()

    stats = {"A": Stats(), "B": Stats()}
    rows = 0
    bad = 0
    t_start = time.time()
    t_sum = t_start
    buf = b""
    print(f"recording {args.port} to {out}; switch on PC log on the LIVE screen. Ctrl+C to stop.")
    with open(out, "w", newline="") as f:
        f.write("t_s,sensor,x_g,y_g,z_g\n")
        try:
            while not args.seconds or time.time() - t_start < args.seconds:
                buf += port.read(8192)
                *lines, buf = buf.split(b"\n")
                for raw in lines:
                    text = ANSI_RE.sub("", raw.decode("utf-8", "replace"))
                    m = CFG_RE.search(text)
                    if m:
                        cfg = dict(kv.split("=", 1) for kv in m.group(1).split() if "=" in kv)
                        if not args.range and "range" in cfg:
                            g_per_count = int(cfg["range"]) / 32768.0
                        if "rate" in cfg:
                            period = 1.0 / int(cfg["rate"])
                        print("board:", m.group(1))
                        continue
                    m = LINE_RE.search(text)
                    if not m:
                        continue
                    s, t0 = m.group(1), int(m.group(2))
                    for item in m.group(4).split():
                        try:
                            dt, x, y, z = (int(p) for p in item.split(","))
                        except ValueError:
                            bad += 1  # a cut-off or garbled entry
                            continue
                        t = (t0 + dt) / 1e6
                        g = (x * g_per_count, y * g_per_count, z * g_per_count)
                        f.write(f"{t:.6f},{s},{g[0]:.5f},{g[1]:.5f},{g[2]:.5f}\n")
                        stats[s].add(t, g, period)
                        rows += 1
                now = time.time()
                if now - t_sum >= args.every:
                    t_sum = now
                    print(f"--- {rows} readings so far, {bad} garbled")
                    for name in ("A", "B"):
                        print(stats[name].line(name))
                        stats[name].reset()
                    sys.stdout.flush()
        except KeyboardInterrupt:
            pass
    port.close()
    print(f"wrote {rows} readings to {out} ({bad} garbled entries skipped)")


if __name__ == "__main__":
    main()
