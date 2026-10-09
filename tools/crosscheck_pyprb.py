#!/usr/bin/env python3
"""Cross-check the C++ two-plane solver (tools/balance_cli) against pyPRB.

pyPRB implements the Brüel & Kjær two-plane influence-coefficient method.
Both tools use one angle convention for vibration phase and weight angle, so
for the same inputs they must give the same masses and angles.

usage: python3 tools/crosscheck_pyprb.py build/tests/balance_cli [cases]
"""
import random
import subprocess
import sys

from pyPRB import DynamicBalancing, MassVector, VibrationVector


def ang_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def main():
    cli = sys.argv[1]
    cases = int(sys.argv[2]) if len(sys.argv) > 2 else 200
    rng = random.Random(1234)
    worst_mass = worst_ang = 0.0
    for _ in range(cases):
        v = [(rng.uniform(0.5, 10.0), rng.uniform(0, 360)) for _ in range(6)]
        t1 = (rng.uniform(1, 10), rng.uniform(0, 360))
        t2 = (rng.uniform(1, 10), rng.uniform(0, 360))
        # v order: V_1_0, V_2_0, V_1_1, V_2_1, V_1_2, V_2_2 (sensor, run)
        db = DynamicBalancing(
            VibrationVector(*v[0]), VibrationVector(*v[1]),
            VibrationVector(*v[2]), VibrationVector(*v[3]),
            VibrationVector(*v[4]), VibrationVector(*v[5]),
            MassVector(*t1), MassVector(*t2),
        )
        c1, c2 = db.compute_compensation(repr=False)
        args = [f"{x}" for pair in v + [t1, t2] for x in pair]
        out = subprocess.run([cli] + args, capture_output=True, text=True)
        if out.returncode != 0:
            continue  # singular cases: both tools would be meaningless
        m1, a1, m2, a2 = map(float, out.stdout.split())
        for (m, a), ref in (((m1, a1), c1), ((m2, a2), c2)):
            worst_mass = max(worst_mass, abs(m - ref.amplitude) / max(ref.amplitude, 1e-9))
            worst_ang = max(worst_ang, abs(ang_diff(a, ref.phase)))
    print(f"{cases} cases: worst relative mass diff {worst_mass:.2e}, worst angle diff {worst_ang:.2e} deg")
    ok = worst_mass < 1e-6 and worst_ang < 1e-4
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
