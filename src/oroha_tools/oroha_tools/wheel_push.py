"""Torque-off hand-push measurement of the wheel constants. NO velocity command is sent.

Two phases so it can be driven from a non-interactive shell while the operator pushes:

  oroha_wheel_push --phase start [--note "payload 0 kg, 200 kPa"]   # torque off both, record counts
      ... operator pushes the robot straight (3 m x 3 m room: along the diagonal) ...
  oroha_wheel_push --phase end --revs 3        # exactly 3 wheel revolutions (tyre mark) -> counts/wheel-rev
  oroha_wheel_push --phase end --dist 2.380    # tape-measured distance -> m/count, diff_cont wheel_radius
  oroha_wheel_push --phase end --revs 3 --dist 2.375   # both: + rolling circumference under load

Both controllers are read (id 1 = RIGHT, id 2 = LEFT). After `start` the wheels roll
freely — chock the robot before and after. Results go to records/measure/ (or
records/tests/<test-id>/); compare with records/calibration/wheel-20261006.yaml
(0.7497 mm/count, 1038.46 counts/wheel rev). Re-run after re-inflating or loading.
Best practice (T20261006-06): reference the robot's FRONT-CENTRE to floor marks a known distance
apart (tyre contact points cannot be located to better than ~1 cm), and push there and back.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

from oroha_tools.ws import records_dir

MD_PORT = "/dev/oroha_md400"
REF_M_PER_COUNT = 0.0007497          # wheel-20261006 (0.7613 = wheel-20260909, no payload)
REF_COUNTS_PER_WHEEL_REV = 1038.46


def pending_path():
    return records_dir() / "measure" / "wheel_push_pending.json"


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


def compute(p0: dict, p1: dict, dist=None, revs=None) -> dict:
    d1, d2 = p1[1] - p0[1], p1[2] - p0[2]
    lin = (abs(d1) + abs(d2)) / 2.0
    res = {"d1": d1, "d2": d2, "lin_counts": lin, "rot_counts": abs(d1) - abs(d2),
           "lr_ratio_abs_d1_d2": abs(d1) / abs(d2) if d2 else None,
           "mirror_sign_ok": (d1 > 0) != (d2 > 0)}
    if dist:
        mpc = dist / lin
        res.update({"m_per_count": mpc, "counts_per_m": 1.0 / mpc,
                    "wheel_radius_effective": mpc * 30.0 / (2 * math.pi),
                    "diff_vs_ref_pct": (mpc / REF_M_PER_COUNT - 1.0) * 100.0})
    if revs:
        cpr = lin / revs
        res.update({"counts_per_wheel_rev": cpr,
                    "cpr_diff_vs_ref_pct": (cpr / REF_COUNTS_PER_WHEEL_REV - 1.0) * 100.0})
        if not dist:
            res["diff_vs_ref_pct"] = res["cpr_diff_vs_ref_pct"]
    if dist and revs:
        res["rolling_circumference_m"] = dist / revs          # ref 0.7906 m (no payload)
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--md-port", default=MD_PORT)
    ap.add_argument("--phase", required=True, choices=["start", "end", "status"])
    ap.add_argument("--dist", type=float, help="tape-measured straight push distance [m] (phase end)")
    ap.add_argument("--revs", type=float, help="whole wheel revolutions pushed (phase end; may be combined with --dist)")
    ap.add_argument("--note", default="", help="conditions: payload, tyre pressure, floor")
    ap.add_argument("--test-id", help="save the result into records/tests/<test-id>/")
    ap.add_argument("--force", action="store_true", help="ignore that the port is open elsewhere")
    a = ap.parse_args(argv)
    pend = pending_path()

    if a.phase == "status":
        print(pend.read_text() if pend.exists() else "no pending measurement")
        return 0

    from oroha_power.port_guard import require_free
    require_free(a.md_port, "oroha_wheel_push", a.force)

    if a.phase == "start":
        torque_off_all(a.md_port)
        p0 = read_positions(a.md_port)
        pend.parent.mkdir(parents=True, exist_ok=True)
        pend.write_text(json.dumps({"start_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                                    "pos_start": p0, "note": a.note}, indent=1))
        print(f"torque OFF (robot rolls freely — chock it). start counts id1 {p0[1]}  id2 {p0[2]}")
        print("push straight to the end mark, stop, chock, then: oroha_wheel_push --phase end --revs N | --dist M")
        return 0

    # end
    if not pend.exists():
        print("no pending start — run --phase start first")
        return 2
    if not (a.dist or a.revs):
        print("phase end needs --revs or --dist")
        return 2
    start = json.loads(pend.read_text())
    p0 = {int(k): v for k, v in start["pos_start"].items()}
    p1 = read_positions(a.md_port)
    res = compute(p0, p1, a.dist, a.revs)
    if res["d1"] == 0 or res["d2"] == 0:
        print(f"a wheel did not move (d1 {res['d1']}, d2 {res['d2']}) — nothing saved, start is kept")
        return 1
    out = {"start_utc": start["start_utc"], "end_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
           "timezone": time.strftime("%Z"),
           "mode": "+".join(k for k, v in (("revs", a.revs), ("dist", a.dist)) if v), "dist_m": a.dist,
           "revs": a.revs, "pos_start": p0, "pos_end": p1, "note": " / ".join(x for x in (start.get("note"), a.note) if x),
           **res}
    print(f"   delta id1 {res['d1']:+d}  id2 {res['d2']:+d}  (mirror signs {'OK' if res['mirror_sign_ok'] else 'SAME SIGN?'})")
    if a.dist:
        print(f"   {res['m_per_count'] * 1e3:.4f} mm/count ({res['diff_vs_ref_pct']:+.2f} % vs {REF_M_PER_COUNT * 1e3:.4f}), "
              f"diff_cont wheel_radius {res['wheel_radius_effective']:.6f} m")
    if a.revs:
        print(f"   {res['counts_per_wheel_rev']:.2f} counts/wheel rev ({res['cpr_diff_vs_ref_pct']:+.2f} % vs 1038.46)")
    if a.dist and a.revs:
        print(f"   rolling circumference {res['rolling_circumference_m']:.4f} m (0.7906 without payload)")
    print(f"   |d1|/|d2| = {res['lr_ratio_abs_d1_d2']:.5f} (pusher steering, not geometry)")
    out_dir = (records_dir() / "tests" / a.test_id) if a.test_id else (records_dir() / "measure")
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / ("wheel_push_" + time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + ".json")
    path.write_text(json.dumps(out, indent=1, ensure_ascii=False))
    pend.unlink()
    print(f"saved {path}\n   robot is TORQUE-OFF — keep it chocked")
    return 0


if __name__ == "__main__":
    sys.exit(main())
