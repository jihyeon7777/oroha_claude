"""Wheel <-> slave id <-> direction check, one controller at a time. THE MOTOR TURNS.

  oroha_direction_check --id 1            # right side expected: +rpm = forward
  oroha_direction_check --id 2            # left side expected:  +rpm = backward (mirrored mount)
  oroha_direction_check --id both --rpm 100 --sec 3

Preconditions (confirmed interactively): wheels lifted off the ground, E-stop within
reach, nobody near the belts. Sends enable() + set_velocity(+rpm) for `sec` seconds,
then 0, stop(), torque_off() — always, also on Ctrl-C. Records the operator's
observation (which side / which direction) to records/preflight/direction_<ts>.json
so 하드웨어_확인표.md §2 can be updated from evidence.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from oroha_tools.ws import records_dir

MD_PORT = "/dev/oroha_md400"
RPM_CAP = 300
SEC_CAP = 10.0
EXPECTED = {1: ("RIGHT", "forward"), 2: ("LEFT", "backward")}


def run_one(port: str, sid: int, rpm: int, sec: float, assume_yes: bool) -> dict:
    from mdrobot import SingleMotorDriver

    exp_side, exp_dir = EXPECTED[sid]
    print(f"\n=== id {sid}: expected {exp_side} wheels, +{rpm} rpm -> {exp_dir} ===")
    if not assume_yes:
        input("Wheels lifted, E-stop in reach, hands clear? [Enter to spin / Ctrl-C to abort] ")
    rec = {"id": sid, "rpm_cmd": rpm, "sec": sec, "rpm_meas": [], "current_a": [],
           "pos_start": None, "pos_end": None, "alarm": []}
    with SingleMotorDriver.open(port, slave_id=sid, timeout=0.3) as d:
        try:
            m = d.read_monitor()
            rec["pos_start"] = m.position
            d.enable()
            d.set_velocity(rpm)
            t_end = time.monotonic() + min(sec, SEC_CAP)
            while time.monotonic() < t_end:
                m = d.read_monitor()
                rec["rpm_meas"].append(m.speed_rpm)
                rec["current_a"].append(m.current_a)
                if m.status.raw:
                    rec["alarm"] = m.status.active
                print(f"\r   rpm {m.speed_rpm:5d}  I {m.current_a if m.current_a is not None else float('nan'):5.2f} A"
                      f"  pos {m.position:8d}", end="", flush=True)
                time.sleep(0.1)
        finally:
            for fn in (lambda: d.set_velocity(0), d.stop, d.torque_off):
                try:
                    fn()
                except Exception as e:  # noqa: BLE001
                    print(f"\n   stop step failed: {e}")
            time.sleep(0.3)
            try:
                rec["pos_end"] = d.read_monitor().position
            except Exception:  # noqa: BLE001
                pass
    print()
    dpos = (rec["pos_end"] - rec["pos_start"]) if rec["pos_end"] is not None else None
    rec["dpos"] = dpos
    settled = rec["rpm_meas"][len(rec["rpm_meas"]) // 2:] or [0]
    rec["rpm_settled_mean"] = sum(settled) / len(settled)
    print(f"   measured rpm (2nd half) {rec['rpm_settled_mean']:.0f}, position delta {dpos}"
          f" -> counts {'increase' if dpos and dpos > 0 else 'decrease' if dpos and dpos < 0 else '?'}")
    if not assume_yes:
        rec["observed_side"] = input("   Which side moved? [left/right] ").strip().lower()
        rec["observed_dir"] = input("   Which way did the wheels roll? [forward/backward] ").strip().lower()
        rec["matches_expected"] = (rec["observed_side"] == exp_side.lower()
                                   and rec["observed_dir"] == exp_dir)
        print(f"   expected {exp_side.lower()}/{exp_dir}: {'MATCH' if rec['matches_expected'] else 'MISMATCH — fix motor_id_L/R or reverse_* in oroha_controllers.yaml'}")
    return rec


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--md-port", default=MD_PORT)
    ap.add_argument("--id", default="both", choices=["1", "2", "both"])
    ap.add_argument("--rpm", type=int, default=100)
    ap.add_argument("--sec", type=float, default=3.0)
    ap.add_argument("--yes", action="store_true", help="no interactive prompts (still spins!)")
    a = ap.parse_args(argv)
    if not 0 < a.rpm <= RPM_CAP:
        print(f"rpm must be 1..{RPM_CAP}")
        return 2
    ids = [1, 2] if a.id == "both" else [int(a.id)]
    results = {"timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
               "port": a.md_port, "runs": []}
    try:
        for sid in ids:
            results["runs"].append(run_one(a.md_port, sid, a.rpm, a.sec, a.yes))
    except KeyboardInterrupt:
        print("\naborted")
    out_dir = records_dir() / "preflight"
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / ("direction_" + time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + ".json")
    with open(out, "w") as f:
        json.dump(results, f, indent=1, ensure_ascii=False, default=str)
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
