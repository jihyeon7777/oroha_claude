"""Operator CLI for the experiment runner.

  oroha_exp run --path square --side 1.5 --v 0.3 --repeats 3 [--series S20260925-01] [--yes]
  oroha_exp run --path straight --length 2.0 --v 0.2
  oroha_exp run --path circle --radius 0.75 --v 0.3 --direction cw
  oroha_exp run --path s_curve --radius 0.6 --v 0.25 --arc-deg 180
  oroha_exp conditions            # edit records/conditions_latest.yaml interactively
  oroha_exp conditions --set surface="epoxy floor" --set floor_slope_deg=0.2     # non-interactive
  oroha_exp gt --run R20260930-101500-straight --x 1.47 --y -0.03 --yaw-deg -1.5  # tape ground truth
  oroha_exp status | abort | note "text"

--rig-state (mock | lifted | on_ground) is required for `run` and written into the conditions.
On the ground: start runs from your own terminal, one run per invocation (--yes with
--repeats > 1 is refused there), keep the E-stop in hand. During a run: [space]/[Esc] or
Ctrl-C abort, [n] note. The runner node (ros2 run oroha_experiment runner) and the robot
(robot.launch.py) must be up; run oroha_preflight first.
"""

from __future__ import annotations

import argparse
import inspect
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

from oroha_msgs.msg import ExperimentStatus
from oroha_msgs.srv import AddNote, ArmExperiment
from oroha_experiment import profiles as P

CONDITION_FIELDS = [
    ("rig_state", "lifted | on_ground | mock", ""),
    ("surface", "floor material / description", "미측정"),
    ("floor_slope_deg", "spirit level across the travel line [deg]", "미측정"),
    ("robot_mass_kg", "robot mass without payload [kg]", "미측정"),
    ("payload_kg", "payload [kg]", "0"),
    ("tyre_pressure_kpa", "tyre pressure L/R [kPa]", "미측정"),
    ("ambient_c", "ambient temperature [C]", "미측정"),
    ("battery_note", "charged? voltage at start", ""),
    ("operator", "who is running", ""),
    ("note", "anything else", ""),
]


def _ws_root() -> Path:
    env = os.environ.get("OROHA_WS")
    if env:
        return Path(env)
    p = Path.cwd().resolve()
    for c in (p, *p.parents):
        if (c / "oroha.repos").exists():
            return c
    return p


def _conditions_path() -> Path:
    return _ws_root() / "records" / "conditions_latest.yaml"


def edit_conditions(path: Path, interactive: bool = True, sets: dict | None = None) -> dict:
    cur = {}
    if path.exists():
        with open(path) as f:
            cur = yaml.safe_load(f) or {}
    for key, val in (sets or {}).items():
        cur[key] = val
    if interactive:
        print("Conditions (Enter keeps the value in brackets; '미측정' = not measured):")
        for key, help_, default in CONDITION_FIELDS:
            old = cur.get(key, default)
            val = input(f"  {key} ({help_}) [{old}]: ").strip()
            cur[key] = val if val else old
    for key, val in (sets or {}).items():      # explicit --set / --rig-state win over prompts
        cur[key] = val
    cur["updated_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(cur, f, allow_unicode=True, sort_keys=False)
    return cur


class Client(Node):
    def __init__(self):
        super().__init__("oroha_exp_cli")
        self.status: ExperimentStatus | None = None
        self.create_subscription(ExperimentStatus, "/oroha_experiment/status", self._on_status, 10)
        self.cli_arm = self.create_client(ArmExperiment, "/oroha_experiment/arm")
        self.cli_start = self.create_client(Trigger, "/oroha_experiment/start")
        self.cli_abort = self.create_client(Trigger, "/oroha_experiment/abort")
        self.cli_note = self.create_client(AddNote, "/oroha_experiment/note")

    def _on_status(self, m):
        self.status = m

    def call(self, cli, req, timeout=20.0):
        if not cli.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"service {cli.srv_name} unavailable — is the runner node up?")
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            raise RuntimeError(f"service {cli.srv_name} timed out")
        return fut.result()

    def wait_status(self, timeout=3.0):
        t0 = time.monotonic()
        while self.status is None and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.status


class KeyReader:
    """Non-blocking single-key reads from the terminal (restored on exit)."""

    def __init__(self):
        self.fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self.saved = None

    def __enter__(self):
        if self.fd is not None:
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def get(self) -> str:
        if self.fd is None:
            return ""
        r, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if r else ""


def path_params(a) -> dict:
    fn = P.PATHS[a.path]
    accepted = set(inspect.signature(fn).parameters)
    cand = {"length": a.length, "radius": a.radius, "side": a.side, "corner": a.corner,
            "angle_deg": getattr(a, "angle_deg", None),
            "turn_w": a.turn_w, "corner_radius": a.corner_radius, "direction": a.direction,
            "arc_deg": a.arc_deg, "first": a.first, "join": a.join, "dwell": a.dwell, "v": a.v,
            "pre_rest": a.pre_rest, "post_rest": a.post_rest}
    return {k: v for k, v in cand.items() if k in accepted and v is not None}


def _parse_sets(items) -> dict:
    out = {}
    for item in items or []:
        if "=" not in item:
            raise SystemExit(f"--set expects key=value, got {item!r}")
        k, v = item.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def run(a) -> int:
    if a.yes and a.rig_state == "on_ground" and a.repeats > 1:
        print("refused: on the ground each run is started by the operator (no --yes with --repeats > 1)")
        return 2
    params = path_params(a)
    profile = P.build(a.path, **params)          # validate locally first
    end = profile.ideal_path()[-1]
    print(f"{a.path} {params}: {profile.duration:.1f} s, path {profile.path_length():.2f} m, "
          f"ideal end x={end.x:.2f} y={end.y:.2f} yaw={end.yaw * 57.3:.0f} deg")

    cond_path = Path(a.conditions) if a.conditions else _conditions_path()
    sets = {"rig_state": a.rig_state, **_parse_sets(a.set)}
    if cond_path.exists() and a.yes:
        age_h = (time.time() - cond_path.stat().st_mtime) / 3600
        if age_h > 12:
            print(f"WARNING: {cond_path} was last edited {age_h:.0f} h ago — are the conditions still true?")
    edit_conditions(cond_path, interactive=not a.yes, sets=sets)

    series = a.series or time.strftime("S%Y%m%d-%H%M%S")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl-C must reach us to abort
    node = Client()
    results = []
    try:
        for i in range(a.repeats):
            cfg = {"path": a.path, "params": params, "repeats": a.repeats, "repeat_index": i,
                   "series_id": series}
            req = ArmExperiment.Request()
            req.run_config_yaml = yaml.safe_dump(cfg)
            req.conditions_yaml = str(cond_path)
            res = node.call(node.cli_arm, req)
            if not res.ok:
                print(f"ARM refused: {res.message}")
                return 1
            print(f"\n[{i + 1}/{a.repeats}] armed {res.run_id} -> {res.run_dir}\n   {res.message}")
            if not a.yes:
                ans = input("   Enter = START, q = cancel: ").strip().lower()
                if ans == "q":
                    node.call(node.cli_abort, Trigger.Request())
                    print("   cancelled")
                    return 1
            r2 = node.call(node.cli_start, Trigger.Request())
            if not r2.success:
                print(f"START refused: {r2.message}")
                return 1
            try:
                final = monitor(node)
            except KeyboardInterrupt:
                print("\n   Ctrl-C -> ABORT")
                try:
                    node.call(node.cli_abort, Trigger.Request(), timeout=5.0)
                except Exception as e:  # noqa: BLE001
                    print(f"   abort call failed ({e}) — diff_cont stops within cmd_vel_timeout; use the E-stop if not")
                results.append((res.run_id, "ABORTED (Ctrl-C)"))
                break
            results.append((res.run_id, final))
            print(f"   -> {final}")
            if i + 1 < a.repeats and a.rest > 0:
                print(f"   resting {a.rest:.0f} s before the next run")
                time.sleep(a.rest)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print("\nseries %s" % series)
    for rid, st in results:
        print(f"  {rid}  {st}")
    return 0 if all(st == "DONE" for _, st in results) else 1


def monitor(node: Client) -> str:
    """Spin until the runner returns to IDLE; keys: space/Esc abort, n note."""
    last_print = 0.0
    final = "?"
    with KeyReader() as keys:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            st = node.status
            if st is None:
                continue
            if st.state in ("DONE", "ABORTED", "FAILED"):
                final = st.state
            if st.state == "IDLE" and final != "?":
                break
            now = time.monotonic()
            if now - last_print > 0.5:
                last_print = now
                sys.stdout.write(f"\r   {st.state:8s} {st.t_run:6.1f}/{st.t_total:5.1f} s "
                                 f"[{st.segment_index:2d}] {st.segment_label:10s} v={st.v_cmd:+.2f} w={st.w_cmd:+.2f} "
                                 f"bag={'on' if st.bag_active else 'off'}   ")
                sys.stdout.flush()
            k = keys.get()
            if k in (" ", "\x1b"):
                print("\n   ABORT requested")
                node.call(node.cli_abort, Trigger.Request())
            elif k == "n":
                keys.__exit__(None, None, None)
                text = input("\n   note: ")
                keys.__enter__()
                if text:
                    rq = AddNote.Request(); rq.text = text
                    node.call(node.cli_note, rq)
    print()
    return final


def write_gt(a) -> int:
    """Manual ground truth (tape) for a run, in the robot's START frame: x forward, y left."""
    run_dir = _ws_root() / "data" / "runs" / a.run
    if not (run_dir / "meta.yaml").exists():
        print(f"no run {a.run} under data/runs")
        return 2
    gt = {"run_id": a.run, "method": a.method, "x_m": a.x, "y_m": a.y, "yaw_deg": a.yaw_deg,
          "frame": "robot start pose: x forward, y left, yaw ccw+", "note": a.note,
          "recorded_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    path = run_dir / "manual_gt.yaml"
    if path.exists() and not a.overwrite:
        print(f"{path} exists; pass --overwrite to replace it")
        return 2
    with open(path, "w") as f:
        yaml.safe_dump(gt, f, allow_unicode=True, sort_keys=False)
    print(f"wrote {path}")
    return 0


def simple(a) -> int:
    rclpy.init()
    node = Client()
    try:
        if a.cmd == "status":
            st = node.wait_status()
            if st is None:
                print("no status — runner not running?")
                return 1
            print(f"{st.state} run={st.run_id} path={st.path} t={st.t_run:.1f}/{st.t_total:.1f} "
                  f"seg={st.segment_label} bag={'on' if st.bag_active else 'off'} preflight_ok={st.preflight_ok}")
        elif a.cmd == "abort":
            r = node.call(node.cli_abort, Trigger.Request())
            print(r.message)
        elif a.cmd == "note":
            rq = AddNote.Request(); rq.text = a.text
            print("ok" if node.call(node.cli_note, rq).ok else "refused (no active run)")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="OROHA experiment CLI")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="arm/start one or more runs")
    r.add_argument("--path", required=True, choices=sorted(P.PATHS))
    r.add_argument("--rig-state", required=True, choices=["mock", "lifted", "on_ground"])
    r.add_argument("--set", action="append", metavar="KEY=VALUE", help="condition field (repeatable)")
    r.add_argument("--angle-deg", type=float, help="spot: turn angle")
    r.add_argument("--v", type=float, default=0.2)
    r.add_argument("--length", type=float)
    r.add_argument("--radius", type=float)
    r.add_argument("--side", type=float)
    r.add_argument("--corner", choices=["spot", "arc"])
    r.add_argument("--turn-w", type=float)
    r.add_argument("--corner-radius", type=float)
    r.add_argument("--direction", choices=["ccw", "cw"])
    r.add_argument("--arc-deg", type=float)
    r.add_argument("--first", choices=["left", "right"])
    r.add_argument("--join", choices=["continuous", "stop"])
    r.add_argument("--dwell", type=float)
    r.add_argument("--pre-rest", type=float)
    r.add_argument("--post-rest", type=float)
    r.add_argument("--repeats", type=int, default=1)
    r.add_argument("--rest", type=float, default=5.0, help="seconds between repeats")
    r.add_argument("--series", help="series id shared by the repeats")
    r.add_argument("--conditions", help="conditions YAML (default records/conditions_latest.yaml)")
    r.add_argument("--yes", action="store_true", help="no prompts (conditions file must exist)")
    c = sub.add_parser("conditions", help="edit the conditions file")
    c.add_argument("--set", action="append", metavar="KEY=VALUE",
                   help="set a field without prompting (repeatable)")
    g = sub.add_parser("gt", help="record the tape-measured end pose of a run")
    g.add_argument("--run", required=True)
    g.add_argument("--x", type=float, required=True, help="m, forward from the start pose")
    g.add_argument("--y", type=float, required=True, help="m, left of the start pose")
    g.add_argument("--yaw-deg", type=float, default=None, help="deg, ccw+ (optional)")
    g.add_argument("--method", default="tape")
    g.add_argument("--note", default="")
    g.add_argument("--overwrite", action="store_true")
    sub.add_parser("status")
    sub.add_parser("abort")
    n = sub.add_parser("note")
    n.add_argument("text")
    a = ap.parse_args(argv)
    if a.cmd == "run":
        return run(a)
    if a.cmd == "conditions":
        sets = _parse_sets(a.set)
        edit_conditions(_conditions_path(), interactive=not sets, sets=sets)
        return 0
    if a.cmd == "gt":
        return write_gt(a)
    return simple(a)


if __name__ == "__main__":
    sys.exit(main())
