"""Torque-off hand-push measurement of the wheel constants. NO velocity command is sent.

  oroha_wheel_push --dist 6.000            # push a tape-measured distance -> m/count, counts/m
  oroha_wheel_push --revs 3                # push exactly N wheel revolutions (mark on tyre) -> counts/wheel-rev

Both controllers are read (id 1 = RIGHT, id 2 = LEFT). torque_off() is sent first so
the wheels roll freely — the robot is then free to roll: chock it before and after.
Writes records/measure/wheel_push_<ts>.json. Compare with records/calibration/wheel-20260909.yaml
(0.7613 mm/count, 1038.46 counts/wheel rev). Re-run after re-inflating or loading.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

from oroha_tools.ws import records_dir

MD_PORT = "/dev/oroha_md400"
REF_M_PER_COUNT = 0.0007613
REF_COUNTS_PER_WHEEL_REV = 1038.46


def read_positions(port: str) -> dict:
    from mdrobot import SingleMotorDriver
    out = {}
    for sid in (1, 2):
        with SingleMotorDriver.open(port, slave_id=sid, timeout=0.3) as d:
            out[sid] = d.read_monitor().position
    return out


def torque_off_all(port: str) -> None:
    from mdrobot import SingleMotorDriver
    for sid in (1, 2):
        with SingleMotorDriver.open(port, slave_id=sid, timeout=0.3) as d:
            d.torque_off()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--md-port", default=MD_PORT)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--dist", type=float, help="tape-measured straight push distance [m]")
    g.add_argument("--revs", type=float, help="whole wheel revolutions pushed (tyre mark)")
    ap.add_argument("--note", default="", help="conditions: payload, tyre pressure, floor")
    a = ap.parse_args(argv)

    print("torque off both controllers — CHOCK the robot, then remove chocks to push")
    torque_off_all(a.md_port)
    input("Robot at the START mark, at rest? [Enter] ")
    p0 = read_positions(a.md_port)
    print(f"   start counts id1 {p0[1]}  id2 {p0[2]}")
    input("Push straight to the END mark, stop, chock. [Enter] ")
    p1 = read_positions(a.md_port)
    d1, d2 = p1[1] - p0[1], p1[2] - p0[2]
    print(f"   end counts   id1 {p1[1]}  id2 {p1[2]}   delta id1 {d1:+d}  id2 {d2:+d}")
    if d1 == 0 or d2 == 0:
        print("   a wheel did not move — aborting")
        return 1
    if (d1 > 0) == (d2 > 0):
        print("   WARNING: both deltas have the same sign; mirrored mount expects opposite signs")
    lin = (abs(d1) + abs(d2)) / 2.0
    rot = (abs(d1) - abs(d2))
    res = {"timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
           "mode": "dist" if a.dist else "revs", "dist_m": a.dist, "revs": a.revs,
           "pos_start": p0, "pos_end": p1, "d1": d1, "d2": d2, "lin_counts": lin,
           "rot_counts": rot, "note": a.note}
    if a.dist:
        mpc = a.dist / lin
        res.update({"m_per_count": mpc, "counts_per_m": 1.0 / mpc,
                    "wheel_radius_effective": mpc * 30.0 / (2 * 3.141592653589793),
                    "ref_m_per_count": REF_M_PER_COUNT,
                    "diff_vs_ref_pct": (mpc / REF_M_PER_COUNT - 1.0) * 100.0})
        print(f"   {mpc * 1e3:.4f} mm/count ({res['diff_vs_ref_pct']:+.2f} % vs 2026-09-05), "
              f"diff_cont wheel_radius = {res['wheel_radius_effective']:.6f} m")
    else:
        cpr = lin / a.revs
        res.update({"counts_per_wheel_rev": cpr, "ref": REF_COUNTS_PER_WHEEL_REV,
                    "diff_vs_ref_pct": (cpr / REF_COUNTS_PER_WHEEL_REV - 1.0) * 100.0})
        print(f"   {cpr:.2f} counts/wheel rev ({res['diff_vs_ref_pct']:+.2f} % vs 1038.46)")
    print(f"   left/right count ratio |d1|/|d2| = {abs(d1) / abs(d2):.5f} (pusher steering, not geometry)")
    out_dir = records_dir() / "measure"
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / ("wheel_push_" + time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + ".json")
    with open(out, "w") as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    print(f"saved {out}\n   robot is TORQUE-OFF — keep it chocked")
    return 0


if __name__ == "__main__":
    sys.exit(main())
