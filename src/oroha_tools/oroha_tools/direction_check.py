"""Wheel <-> slave id <-> direction check, one controller at a time. THE MOTOR TURNS.

  oroha_direction_check --id 1 --yes                     # expected: RIGHT wheels forward
  oroha_direction_check --id 2 --yes                     # expected: LEFT wheels backward (+rpm, mirrored)
  oroha_direction_check --id 1 --sec 10 --yes --test-id T20260928-06    # E-stop characterisation (E1)
  oroha_direction_check --id 1 --sec 10 --resend --yes   # E1 with the command re-sent every poll (as ros2_control does)

Preconditions: wheels lifted, E-stop in the operator's hand, nobody near the belts.
From a non-interactive shell pass --yes only after the operator said "go"; the operator's
observation can be recorded with --observed-side/--observed-dir (or written into the
test notes afterwards).

Sequence per id: read version / USE_LIMIT_SW / ENC_PPR -> set USE_LIMIT_SW to the runtime
value (0, same as the ros2_control plugin) -> enable() -> set_velocity(+rpm) -> poll the
monitor (rpm, current, position, status) every ~0.1 s for --sec seconds -> ALWAYS
set_velocity(0), stop(), torque_off() — also on Ctrl-C and SIGTERM.

The JSON (records/preflight/ or records/tests/<test-id>/) holds the timed samples and an
automatic reading of stop/resume segments, which is what the E-stop test needs.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from pathlib import Path

from oroha_tools.ws import records_dir

MD_PORT = "/dev/oroha_md400"
RPM_CAP = 300
SEC_CAP = 10.0
EXPECTED = {1: ("right", "forward"), 2: ("left", "backward")}
STOP_RPM = 5          # |rpm| below this while commanded = "not turning"


def _sigterm(signum, frame):
    raise KeyboardInterrupt


def analyse(samples, rpm_cmd) -> dict:
    """Find segments where the wheel stopped while still commanded and whether it resumed."""
    moving = [abs(s["rpm"]) >= STOP_RPM for s in samples]
    segs = []
    started = False
    stop_t = None
    for s, m in zip(samples, moving):
        if m:
            started = True
            if stop_t is not None:
                segs[-1]["resumed_t"] = s["t"]
                stop_t = None
        elif started and stop_t is None:
            stop_t = s["t"]
            segs.append({"stopped_t": s["t"], "resumed_t": None})
    first_move = next((s["t"] for s, m in zip(samples, moving) if m), None)
    return {"first_motion_t": first_move, "stop_segments": segs,
            "resumed_after_stop": any(sg["resumed_t"] is not None for sg in segs),
            "rpm_cmd": rpm_cmd}


def run_one(port: str, sid: int, rpm: int, sec: float, resend: bool, use_limit_sw: str,
            confirm: bool) -> dict:
    from mdrobot import SingleMotorDriver
    from mdrobot import registers as reg

    exp_side, exp_dir = EXPECTED[sid]
    print(f"\n=== id {sid}: expected {exp_side.upper()} wheels, +{rpm} rpm -> {exp_dir} "
          f"({sec:.0f} s{', re-sent every poll' if resend else ''}) ===")
    if confirm:
        input("Wheels lifted, E-stop in hand, hands clear? [Enter = spin, Ctrl-C = abort] ")
    rec = {"id": sid, "rpm_cmd": rpm, "sec": sec, "resend": resend, "samples": []}
    with SingleMotorDriver.open(port, slave_id=sid, timeout=0.3) as d:
        try:
            rec["version"] = d.get_version()
            rec["use_limit_sw_before"] = d.client.read_register(reg.PID_USE_LIMIT_SW)
            rec["enc_ppr"] = d.client.read_register(reg.PID_ENC_PPR)
            if use_limit_sw != "keep":
                d.client.write_register(reg.PID_USE_LIMIT_SW, int(use_limit_sw))
            rec["use_limit_sw_used"] = d.client.read_register(reg.PID_USE_LIMIT_SW)
            print(f"   fw v{rec['version']}  USE_LIMIT_SW {rec['use_limit_sw_before']} -> "
                  f"{rec['use_limit_sw_used']}  ENC_PPR {rec['enc_ppr']} "
                  f"({'hall, 30 counts/rev' if rec['enc_ppr'] == 0 else 'ENCODER MODE — counts_per_rev 30 invalid'})")
            m = d.read_monitor()
            rec["pos_start"] = m.position
            d.enable()
            t0 = time.monotonic()
            rec["cmd_wall_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            d.set_velocity(rpm)
            while (t := time.monotonic() - t0) < min(sec, SEC_CAP):
                if resend:
                    d.set_velocity(rpm)
                m = d.read_monitor()
                s = {"t": round(t, 3), "rpm": m.speed_rpm,
                     "current_a": m.current_a, "pos": m.position,
                     "status": m.status.raw, "alarms": m.status.active}
                rec["samples"].append(s)
                print(f"\r   t {t:5.1f}s  rpm {m.speed_rpm:5d}  I {s['current_a'] if s['current_a'] is not None else float('nan'):5.2f} A"
                      f"  pos {m.position:8d}  status {m.status.raw:3d}   ", end="", flush=True)
                time.sleep(0.1)
        finally:
            for name, fn in (("set_velocity(0)", lambda: d.set_velocity(0)),
                             ("stop", d.stop), ("torque_off", d.torque_off)):
                try:
                    fn()
                except Exception as e:  # noqa: BLE001
                    print(f"\n   {name} failed: {e} — USE THE E-STOP / oroha_md_stop")
            time.sleep(0.3)
            try:
                rec["pos_end"] = d.read_monitor().position
            except Exception:  # noqa: BLE001
                rec["pos_end"] = None
    print()
    rec["dpos"] = (rec["pos_end"] - rec["pos_start"]) if rec.get("pos_end") is not None else None
    rec["analysis"] = analyse(rec["samples"], rpm)
    a = rec["analysis"]
    print(f"   position delta {rec['dpos']} counts; first motion {a['first_motion_t']} s; "
          f"stop segments {a['stop_segments']}; resumed after stop: {a['resumed_after_stop']}")
    return rec


def main(argv=None) -> int:
    signal.signal(signal.SIGTERM, _sigterm)
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--md-port", default=MD_PORT)
    ap.add_argument("--id", default="both", choices=["1", "2", "both"])
    ap.add_argument("--rpm", type=int, default=100)
    ap.add_argument("--sec", type=float, default=3.0)
    ap.add_argument("--resend", action="store_true", help="re-send the command every poll")
    ap.add_argument("--use-limit-sw", default="0", choices=["0", "1", "keep"],
                    help="value written before the run (0 = runtime value of the plugin)")
    ap.add_argument("--yes", action="store_true", help="no prompt (operator said go)")
    ap.add_argument("--observed-side", help="operator report, e.g. right or right,left for --id both")
    ap.add_argument("--observed-dir", help="operator report, e.g. forward or forward,backward")
    ap.add_argument("--test-id", help="save into records/tests/<test-id>/")
    ap.add_argument("--note", default="")
    ap.add_argument("--force", action="store_true", help="ignore that the port is open elsewhere")
    a = ap.parse_args(argv)
    if not 0 < a.rpm <= RPM_CAP or not 0 < a.sec <= SEC_CAP:
        print(f"rpm must be 1..{RPM_CAP}, sec 0..{SEC_CAP:.0f}")
        return 2
    if not a.yes and not sys.stdin.isatty():
        print("no terminal: pass --yes once the operator has confirmed wheels lifted + E-stop in hand")
        return 2
    from oroha_power.port_guard import require_free
    require_free(a.md_port, "oroha_direction_check", a.force)

    ids = [1, 2] if a.id == "both" else [int(a.id)]
    obs_side = (a.observed_side or "").split(",") if a.observed_side else []
    obs_dir = (a.observed_dir or "").split(",") if a.observed_dir else []
    out = {"timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
           "timezone": time.strftime("%Z"), "port": a.md_port, "args": vars(a), "runs": []}
    try:
        for k, sid in enumerate(ids):
            rec = run_one(a.md_port, sid, a.rpm, a.sec, a.resend, a.use_limit_sw, not a.yes)
            side = obs_side[k].strip().lower() if k < len(obs_side) else None
            direction = obs_dir[k].strip().lower() if k < len(obs_dir) else None
            if not a.yes and sys.stdin.isatty():
                side = side or input("   Which side moved? [left/right] ").strip().lower()
                direction = direction or input("   Which way did the wheels roll? [forward/backward] ").strip().lower()
            rec["observed_side"], rec["observed_dir"] = side, direction
            if side and direction:
                exp = EXPECTED[sid]
                rec["matches_expected"] = (side, direction) == exp
                print(f"   observed {side}/{direction}, expected {exp[0]}/{exp[1]}: "
                      f"{'MATCH' if rec['matches_expected'] else 'MISMATCH — fix motor_id_L/R or reverse_* in oroha_controllers.yaml'}")
            out["runs"].append(rec)
    except KeyboardInterrupt:
        print("\naborted (motor stop sequence already ran)")
        out["aborted"] = True
    out_dir = (records_dir() / "tests" / a.test_id) if a.test_id else (records_dir() / "preflight")
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / ("direction_" + time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + ".json")
    with open(path, "w") as f:
        json.dump(out, f, indent=1, ensure_ascii=False, default=str)
    print(f"saved {path}")
    return 1 if out.get("aborted") else 0


if __name__ == "__main__":
    sys.exit(main())
