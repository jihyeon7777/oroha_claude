"""Experiment runner: arm -> start -> (events, bag) -> done/abort/fail, one run at a time.

Node name `oroha_experiment`. Services
  ~/arm    oroha_msgs/ArmExperiment  build + validate the profile, run checks, create the run dir,
                                     write meta/profile/versions, start `ros2 bag record`, hold zeros
  ~/start  std_srvs/Trigger          begin the time-based profile (RUNNING)
  ~/abort  std_srvs/Trigger          ramp to zero (1 s of zeros), ABORTED
  ~/note   oroha_msgs/AddNote        NOTE event with free text
Topics
  <cmd_topic>  geometry_msgs/TwistStamped at cmd_rate while ARMED (zeros) / RUNNING
  ~/event      oroha_msgs/ExperimentEvent (RELIABLE, TRANSIENT_LOCAL)
  ~/status     oroha_msgs/ExperimentStatus 5 Hz
Guards while RUNNING: per-wheel motion guard (guards.py: a side not turning / turning the wrong
way for > stall_timeout_s -> FAIL, with Pico currents and MD400 status bits as evidence: E-stop vs
blocked wheel vs lost MD400), no /joint_states (FAIL + direct Modbus stop), bag process death (FAIL).

Events beyond the run's own (M4): PREFLIGHT_OK at ARM, ZERO when oroha_power applies a '#ZERO',
MANUAL_MOVE from a note starting with "MANUAL_MOVE", and NOTE "log <node> WARN/ERROR: ..." mirrored
from /rosout (driver comm retries, power reconnects) while a run is armed/running — the bag is not
committed, events.csv is, so the committed record shows what disturbed a run.

Run directory  data/runs/R<YYYYMMDD>-<HHMMSS>-<path>/
  meta.yaml  profile.csv  events.csv  conditions.yaml  versions.yaml  uncommitted.diff  untracked.txt
  params.yaml (M3: parameters of the robot nodes + hardware component states at ARM)  bag/
"""

from __future__ import annotations

import csv
import math
import os
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path

import atexit

import rclpy
import yaml
from geometry_msgs.msg import TwistStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rcl_interfaces.msg import Log
from std_msgs.msg import String
from std_srvs.srv import Trigger
from control_msgs.msg import DynamicJointState

from oroha_msgs.msg import ExperimentEvent, ExperimentStatus
from oroha_msgs.srv import AddNote, ArmExperiment
from oroha_experiment import profiles as P
from oroha_experiment.guards import WheelGuard, describe

DEFAULT_BAG_TOPICS = [
    "/diff_cont/cmd_vel", "/diff_cont/cmd_vel_out", "/diff_cont/odom", "/joint_states",
    "/dynamic_joint_states",
    "/tf", "/tf_static", "/robot_description",
    "/oroha_power/sample", "/oroha_power/battery", "/oroha_power/calibration_event",
    "/imu/data", "/imu/mag", "/imu/temperature", "/diagnostics",
    "/oroha_experiment/event", "/oroha_experiment/status", "/rosout",
]
STATES = ("IDLE", "ARMED", "RUNNING", "DONE", "ABORTED", "FAILED")
# nodes whose parameters are snapshotted at ARM (params.yaml)
PARAM_NODES = ["/controller_manager", "/diff_cont", "/joint_state_broadcaster", "/robot_state_publisher",
               "/oroha_power", "/um7_node"]
# /rosout loggers mirrored as NOTE events (WARN and above) while a run is active
LOG_MIRROR = ["MdrobotSystemHardware", "controller_manager", "resource_manager", "diff_cont",
              "oroha_power", "um7_node"]


def _ws_root() -> Path:
    env = os.environ.get("OROHA_WS")
    if env:
        return Path(env)
    p = Path.cwd().resolve()
    for c in (p, *p.parents):
        if (c / "oroha.repos").exists():
            return c
    return p


class ExperimentRunner(Node):

    def __init__(self):
        super().__init__("oroha_experiment")
        d = self.declare_parameter
        d("cmd_topic", "/diff_cont/cmd_vel")
        d("cmd_rate", 20.0)
        d("frame_id", "base_link")
        d("runs_dir", "")
        d("record_bag", True)
        d("bag_storage", "mcap")
        d("bag_topics", DEFAULT_BAG_TOPICS)
        d("require_preflight", True)
        d("preflight_max_age_s", 6.0 * 3600)
        d("require_controller", True)
        d("controller_name", "diff_cont")
        d("require_power", True)
        d("power_diag_name", "oroha_power: Pico current/voltage")
        d("min_free_gb", 1.0)
        d("stall_enable", True)
        d("stall_timeout_s", 1.0)
        d("stall_min_expected", 10.0)   # motor rad/s a side must be asked for before it is checked
        d("stall_motor_rad_s", 2.0)     # measured motor-shaft speed below this = not turning
        d("left_joint", "motor_L")
        d("right_joint", "motor_R")
        d("wheel_radius", 0.003580)     # defaults; replaced by diff_cont's values at ARM when readable
        d("wheel_separation", 0.451)
        d("log_mirror_max", 50)         # NOTE events from /rosout per run
        d("abort_zero_s", 1.0)
        d("direct_stop_on_fail", True)      # Modbus stop of both MD400 after a FAIL (hardware link lost)
        d("md_port", "/dev/oroha_md400")
        # first ground sessions (plan review C5): diff_cont keeps its own hard caps (0.8 / 2.0)
        d("v_max", 0.35)
        d("w_max", 1.2)
        d("arena_m", 3.0)               # square room side
        d("arena_margin_m", 0.3)        # drift allowance at every wall
        d("robot_length_m", P.BODY_LENGTH)   # outer footprint incl. tyres, measured (T20261006-05)
        d("robot_width_m", P.BODY_WIDTH)

        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.cmd_topic = str(g("cmd_topic"))
        self.cmd_rate = float(g("cmd_rate"))
        self.frame_id = str(g("frame_id"))
        self.runs_dir = Path(str(g("runs_dir")) or (_ws_root() / "data" / "runs"))
        self.records_dir = _ws_root() / "records"

        self.state = "IDLE"
        self.lock = threading.RLock()
        self.profile: P.Profile | None = None
        self.run_id = ""
        self.run_dir: Path | None = None
        self.meta: dict = {}
        self.t_start_ns = 0
        self.seg_idx = -1
        self.cmd_v = self.cmd_w = 0.0
        self.bag_proc: subprocess.Popen | None = None
        self.events: list = []
        self.repeat_index = -1
        self.repeats = 1
        self.zero_until_ns = 0
        self.preflight_ok = False
        # guards
        self.joint_vel: dict = {}
        self.last_joint_ns = 0
        self.stall_since_ns = 0
        self.power_ok_ns = 0
        self.currents = None            # (i_left, i_right) latest Pico sample, A vs true 0 A
        self.md_status = {}             # joint -> MD400 status byte (patch 0001)
        self.guard = WheelGuard()
        self.log_seen: dict = {}
        self.n_log_mirrored = 0

        latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        # RELIABLE matches both RELIABLE and BEST_EFFORT subscriptions; a BEST_EFFORT publisher
        # would be ignored by a RELIABLE diff_cont subscription (plan review H7).
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_cmd = self.create_publisher(TwistStamped, self.cmd_topic, cmd_qos)
        self.pub_event = self.create_publisher(ExperimentEvent, "~/event", latched)
        self.pub_status = self.create_publisher(ExperimentStatus, "~/status", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diag, 10)
        self.create_subscription(DynamicJointState, "/dynamic_joint_states", self.on_dyn_joint, 10)
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        from oroha_msgs.msg import PowerSample
        self.create_subscription(PowerSample, "/oroha_power/sample", self.on_power, sensor)
        self.create_subscription(String, "/oroha_power/calibration_event", self.on_cal_event, latched)
        self.create_subscription(Log, "/rosout", self.on_rosout, 100)

        self.srv_group = ReentrantCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.create_service(ArmExperiment, "~/arm", self.on_arm, callback_group=self.srv_group)
        self.create_service(Trigger, "~/start", self.on_start, callback_group=self.srv_group)
        self.create_service(Trigger, "~/abort", self.on_abort, callback_group=self.srv_group)
        self.create_service(AddNote, "~/note", self.on_note, callback_group=self.srv_group)
        self.create_timer(1.0 / self.cmd_rate, self.tick, callback_group=self.timer_group)
        self.create_timer(0.2, self.pub_status_msg, callback_group=self.timer_group)
        self.get_logger().info("experiment runner ready; runs -> %s" % self.runs_dir)

    # ------------------------------------------------------------- helpers
    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def t_run(self) -> float:
        return (self.now_ns() - self.t_start_ns) * 1e-9 if self.t_start_ns else -1.0

    def event(self, kind: str, detail: str = "", seg: int = -1):
        m = ExperimentEvent()
        m.run_id = self.run_id
        m.event = kind
        m.detail = detail
        m.repeat_index = self.repeat_index
        m.segment_index = seg
        m.t_run = self.t_run() if self.state in ("RUNNING",) or kind in ("END", "ABORT", "FAIL") else -1.0
        try:
            m.header.stamp = self.get_clock().now().to_msg()
            self.pub_event.publish(m)
        except Exception:  # noqa: BLE001 - still written to events.csv below
            pass
        row = {"stamp_ns": time.time_ns(), "t_run": round(m.t_run, 4), "event": kind,
               "segment_index": seg, "detail": detail}
        self.events.append(row)
        if self.run_dir:
            path = self.run_dir / "events.csv"
            new = not path.exists()
            with open(path, "a", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(row.keys()))
                if new:
                    w.writeheader()
                w.writerow(row)
        try:
            self.get_logger().info("[%s] %s %s" % (self.run_id, kind, detail))
        except Exception:  # noqa: BLE001
            print("[%s] %s %s" % (self.run_id, kind, detail))

    def publish_cmd(self, v: float, w: float):
        self.cmd_v, self.cmd_w = v, w
        try:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.frame_id
            m.twist.linear.x = float(v)
            m.twist.angular.z = float(w)
            self.pub_cmd.publish(m)
        except Exception:  # noqa: BLE001 - context may be gone during shutdown; diff_cont times out
            pass

    # -------------------------------------------------------------- inputs
    def on_joint_states(self, msg: JointState):
        if msg.velocity:
            self.joint_vel = dict(zip(msg.name, msg.velocity))
            self.last_joint_ns = self.now_ns()

    def on_dyn_joint(self, msg: DynamicJointState):
        for jn, iv in zip(msg.joint_names, msg.interface_values):
            vals = dict(zip(iv.interface_names, iv.values))
            if "status" in vals:
                self.md_status[jn] = int(vals["status"])

    def on_power(self, msg):
        self.currents = (float(msg.i_left), float(msg.i_right))

    def _active(self) -> bool:
        return self.state in ("ARMED", "RUNNING", "DONE", "ABORTED", "FAILED") and self.run_dir is not None

    def on_cal_event(self, msg: String):
        # "<ns> text": mirror applied zeros into the run (M4); calibration loads are not events
        text = msg.data.split(" ", 1)[-1]
        with self.lock:
            if self._active() and (text.startswith("applied:") or text.startswith("#ZERO")):
                if int(msg.data.split(" ", 1)[0]) >= self.meta.get("armed_ros_ns", 0):
                    self.event(ExperimentEvent.ZERO, text[:200], self.seg_idx)

    def on_rosout(self, msg: Log):
        if msg.level < Log.WARN or msg.name not in LOG_MIRROR:
            return
        with self.lock:
            if not self._active():
                return
            stamp = msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec
            if stamp < self.meta.get("armed_ros_ns", 0):
                return                      # logged before this run (rosout history / late delivery)
            if "Velocity command timed out" in msg.msg and (self.state != "RUNNING" or stamp < self.t_start_ns):
                return                      # zeros pause while ARM starts the bag; harmless before START
            key = (msg.name, msg.msg[:40])
            now = self.now_ns()
            if now - self.log_seen.get(key, 0) < 5e9:      # same message at most every 5 s
                return
            self.log_seen[key] = now
            self.n_log_mirrored += 1
            self.meta["log_warnings"] = self.n_log_mirrored
            if self.n_log_mirrored <= int(self.get_parameter("log_mirror_max").value):
                lvl = {Log.WARN: "WARN", Log.ERROR: "ERROR", Log.FATAL: "FATAL"}.get(msg.level, str(msg.level))
                self.event(ExperimentEvent.NOTE, "log %s %s: %s" % (msg.name, lvl, msg.msg[:200]), self.seg_idx)

    def on_diag(self, msg: DiagnosticArray):
        name = str(self.get_parameter("power_diag_name").value)
        for st in msg.status:
            if st.name == name and st.level in (DiagnosticStatus.OK, DiagnosticStatus.WARN):
                self.power_ok_ns = self.now_ns()

    # -------------------------------------------------------------- checks
    def _latest_preflight(self):
        d = self.records_dir / "preflight"
        files = sorted(d.glob("2*.json")) if d.exists() else []
        return files[-1] if files else None

    def checks(self) -> list:
        """Return a list of failure strings (empty = ok)."""
        import json
        fails = []
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.preflight_ok = False
        if g("require_preflight"):
            pf = self._latest_preflight()
            if pf is None:
                fails.append("no preflight result in records/preflight (run oroha_preflight)")
            else:
                age = time.time() - pf.stat().st_mtime
                try:
                    ok = json.load(open(pf)).get("ok", False)
                except Exception:  # noqa: BLE001
                    ok = False
                if not ok:
                    fails.append("latest preflight %s did not pass" % pf.name)
                elif age > float(g("preflight_max_age_s")):
                    fails.append("latest preflight %s is %.1f h old" % (pf.name, age / 3600))
                else:
                    self.preflight_ok = True
                    self.meta["preflight"] = str(pf)
        if g("require_controller"):
            active = self._controller_active(str(g("controller_name")))
            if active is not True:
                fails.append("controller %s not active (%s)" % (g("controller_name"), active))
        if g("require_power"):
            if (self.now_ns() - self.power_ok_ns) * 1e-9 > 3.0:
                fails.append("oroha_power diagnostics not OK within 3 s")
        others = [p.node_name for p in self.get_publishers_info_by_topic(self.cmd_topic)
                  if p.node_name != self.get_name()]
        if others:
            fails.append("other publishers on %s: %s (stop teleop)" % (self.cmd_topic, others))
        free_gb = shutil.disk_usage(str(self.runs_dir.parent if self.runs_dir.exists() else _ws_root())).free / 1e9
        if free_gb < float(g("min_free_gb")):
            fails.append("free disk %.2f GB < %.1f GB" % (free_gb, g("min_free_gb")))
        return fails

    def _controller_active(self, name: str):
        try:
            from controller_manager_msgs.srv import ListControllers
        except ImportError:
            return "controller_manager_msgs missing"
        cli = self.create_client(ListControllers, "/controller_manager/list_controllers",
                                 callback_group=self.srv_group)
        try:
            if not cli.wait_for_service(timeout_sec=2.0):
                return "controller_manager service unavailable"
            fut = cli.call_async(ListControllers.Request())
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 3.0:
                time.sleep(0.05)
            if not fut.done():
                return "list_controllers timeout"
            for c in fut.result().controller:
                if c.name == name:
                    return True if c.state == "active" else c.state
            return "not loaded"
        finally:
            self.destroy_client(cli)

    # ------------------------------------------------------------ services
    def on_arm(self, req, res):
        with self.lock:
            if self.state not in ("IDLE",):
                res.ok, res.message = False, "state is %s, not IDLE" % self.state
                return res
            try:
                cfg = self._load_yaml_or_inline(req.run_config_yaml)
                conditions = self._load_yaml_or_inline(req.conditions_yaml) if req.conditions_yaml else {}
                path = cfg["path"]
                params = dict(cfg.get("params", {}))
                limits = P.Limits(v_max=float(self.get_parameter("v_max").value),
                                  w_max=float(self.get_parameter("w_max").value))
                profile = P.build(path, limits=limits, **params)
            except Exception as e:  # noqa: BLE001
                res.ok, res.message = False, "bad run config: %s" % e
                return res
            rig_state = str(conditions.get("rig_state", "")).strip()
            if rig_state not in ("mock", "lifted", "on_ground"):
                res.ok, res.message = False, ("conditions.rig_state must be mock | lifted | on_ground "
                                              "(oroha_exp run --rig-state ...), got %r" % rig_state)
                return res
            arena = float(self.get_parameter("arena_m").value)
            margin = float(self.get_parameter("arena_margin_m").value)
            body = (float(self.get_parameter("robot_length_m").value), float(self.get_parameter("robot_width_m").value))
            fp = P.footprint(profile, body)
            placement = fp.placement(arena, margin)
            if rig_state == "on_ground" and placement == "none":
                dx, dy = fp.size
                res.ok, res.message = False, ("path needs %.2f x %.2f m, does not fit the %.1f m room with "
                                              "%.1f m margins — make it smaller" % (dx, dy, arena, margin))
                return res
            self.meta = {}
            fails = self.checks() + self._orphan_bags()
            if fails:
                res.ok, res.message = False, "; ".join(fails)
                return res

            self.profile = profile
            self.repeat_index = int(cfg.get("repeat_index", 0))
            self.repeats = int(cfg.get("repeats", 1))
            stamp = time.strftime("%Y%m%d-%H%M%S")
            self.run_id = "R%s-%s" % (stamp, path)
            self.run_dir = self.runs_dir / self.run_id
            self.run_dir.mkdir(parents=True, exist_ok=False)
            self.events = []
            self.t_start_ns = 0
            self.seg_idx = -1
            profile.write_csv(str(self.run_dir / "profile.csv"))
            with open(self.run_dir / "conditions.yaml", "w") as f:
                yaml.safe_dump(conditions, f, allow_unicode=True, sort_keys=False)
            versions = self._snapshot_versions()
            self.meta.update({
                "run_id": self.run_id, "series_id": cfg.get("series_id", ""),
                "repeat_index": self.repeat_index, "repeats": self.repeats,
                "path": path, "params": params, "profile": profile.describe(),
                "timezone": time.strftime("%Z"),
                "armed_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "armed_ros_ns": self.now_ns(),
                "armed_local": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
                "status": "ARMED", "conditions": conditions,
                "versions": versions, "cmd_topic": self.cmd_topic, "cmd_rate_hz": self.cmd_rate,
                "bag": None, "manual_gt": None, "notes": [],
                "rig_state": rig_state,
                "arena": {"room_m": arena, "margin_m": margin, "body_m": list(body),
                          "needed_m": [round(x, 3) for x in fp.size],
                          "placement": placement,
                          "start_offset_m": [round(x, 3) for x in fp.start_offset(margin)]},
            })
            self.log_seen.clear()
            self.n_log_mirrored = 0
            self.meta.update(self._snapshot_params())
            self._configure_guard()
            self._write_meta()
            self.state = "ARMED"
            self.event(ExperimentEvent.ARMED, "%s %s" % (path, params))
            if self.preflight_ok:
                self.event(ExperimentEvent.PREFLIGHT_OK, Path(self.meta["preflight"]).name)
            if self.get_parameter("record_bag").value:
                if not self._start_bag():
                    self._finalize("FAILED", "bag record did not start")
                    res.ok, res.message = False, "bag record did not start"
                    return res
            res.ok, res.run_id, res.run_dir = True, self.run_id, str(self.run_dir)
            ox, oy = fp.start_offset(margin)
            where = {"wall": "start %.2f m from the wall behind, %.2f m from the wall on the right" % (ox, oy),
                     "diagonal": "start near a corner heading along the room diagonal",
                     "none": "room check not applied (%s)" % rig_state}[placement]
            res.message = "armed: %.1f s profile, %d segments; %s" % (
                profile.duration, len(profile.segments), where)
            return res

    def on_start(self, req, res):
        with self.lock:
            if self.state != "ARMED":
                res.success, res.message = False, "state is %s, not ARMED" % self.state
                return res
            self.t_start_ns = self.now_ns()
            self.stall_since_ns = 0
            self.guard.reset()
            self.state = "RUNNING"
            self.meta["start_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            self.meta["start_ros_ns"] = self.t_start_ns
            self.meta["status"] = "RUNNING"
            self._write_meta()
            self.event(ExperimentEvent.START, "t_total=%.2f" % self.profile.duration)
            res.success, res.message = True, self.run_id
            return res

    def on_abort(self, req, res):
        with self.lock:
            if self.state not in ("ARMED", "RUNNING"):
                res.success, res.message = False, "state is %s" % self.state
                return res
            self._begin_stop("ABORTED", "abort requested")
            res.success, res.message = True, "aborting"
            return res

    def on_note(self, req, res):
        with self.lock:
            if self.state == "IDLE":
                res.ok = False
                return res
            self.meta.setdefault("notes", []).append({"t_run": round(self.t_run(), 3), "text": req.text})
            if req.text.upper().startswith("MANUAL_MOVE"):
                # the robot was moved by hand: odometry and the path after this point are not comparable
                self.event(ExperimentEvent.MANUAL_MOVE, req.text[len("MANUAL_MOVE"):].lstrip(" :"), self.seg_idx)
            else:
                self.event(ExperimentEvent.NOTE, req.text, self.seg_idx)
            res.ok = True
            return res

    # ------------------------------------------------------------- run loop
    def tick(self):
        with self.lock:
            if self.state == "ARMED":
                self.publish_cmd(0.0, 0.0)
                return
            if self.state in ("DONE", "ABORTED", "FAILED"):
                # zero hold after stop, then finalize
                self.publish_cmd(0.0, 0.0)
                if self.now_ns() >= self.zero_until_ns:
                    self._finalize(self.state, "")
                return
            if self.state != "RUNNING":
                return
            t = self.t_run()
            v, w, idx, label = self.profile.cmd_at(t)
            if idx != self.seg_idx and idx >= 0:
                self.seg_idx = idx
                self.event(ExperimentEvent.SEGMENT, label, idx)
            self.publish_cmd(v, w)
            if t >= self.profile.duration:
                self.event(ExperimentEvent.END, "profile complete", self.seg_idx)
                self._begin_stop("DONE", "")
                return
            # guards
            if self.bag_proc is not None and self.bag_proc.poll() is not None:
                self.event(ExperimentEvent.FAIL, "bag process exited (%s)" % self.bag_proc.returncode, idx)
                self._begin_stop("FAILED", "bag died")
                return
            if self.get_parameter("stall_enable").value:
                g = self.guard
                el, er = g.expected(v, w)
                moving_cmd = max(abs(el), abs(er)) >= g.min_expected
                fresh = (self.now_ns() - self.last_joint_ns) * 1e-9 < 0.5
                if moving_cmd and not fresh:
                    # hardware link lost: ros2_control stopped publishing (direct stop follows)
                    if self.stall_since_ns == 0:
                        self.stall_since_ns = self.now_ns()
                    elif (self.now_ns() - self.stall_since_ns) * 1e-9 > g.timeout:
                        why = "no /joint_states"
                        self.event(ExperimentEvent.FAIL, why, idx)
                        self._begin_stop("FAILED", why)
                        return
                    return
                self.stall_since_ns = 0
                if fresh:
                    jl, jr = str(self.get_parameter("left_joint").value), str(self.get_parameter("right_joint").value)
                    verdict = g.update(t, v, w, self.joint_vel.get(jl, 0.0), self.joint_vel.get(jr, 0.0))
                    if verdict:
                        st = (self.md_status.get(jl, 0), self.md_status.get(jr, 0)) if self.md_status else None
                        why = describe(verdict, self.currents, st)
                        self.event(ExperimentEvent.FAIL, why, idx)
                        self._begin_stop("FAILED", why)
                        return

    def _begin_stop(self, final_state: str, why: str):
        if final_state == "ABORTED":
            self.event(ExperimentEvent.ABORT, why, self.seg_idx)
        if (final_state == "FAILED" and "no /joint_states" in why
                and self.get_parameter("direct_stop_on_fail").value):
            # When the failure is a lost hardware link, ros2_control has deactivated the
            # component and no zero will ever reach the MD400s, which keep their last
            # command (T20260929-05: right wheel kept turning after a left-MD400 power
            # dip). Stop them directly over Modbus, retrying while the bus recovers.
            threading.Thread(target=self._direct_stop, daemon=True).start()
        self.state = final_state
        self.zero_until_ns = self.now_ns() + int(float(self.get_parameter("abort_zero_s").value) * 1e9)
        self.publish_cmd(0.0, 0.0)

    def _finalize(self, final_state: str, why: str):
        try:
            self._stop_bag()
        except Exception as e:  # noqa: BLE001 - never skip the meta write
            print("bag stop failed: %s" % e)
        self.meta["status"] = final_state
        self.meta["end_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        self.meta["t_run_end"] = round(self.t_run(), 3)
        if self.run_dir and (self.run_dir / "bag").exists():
            size = sum(p.stat().st_size for p in (self.run_dir / "bag").rglob("*") if p.is_file())
            self.meta["bag"] = {"path": "bag", "bytes": size,
                                "topics": list(self.get_parameter("bag_topics").value)}
        self._write_meta()
        self.get_logger().info("run %s finished: %s" % (self.run_id, final_state))
        self.state = "IDLE"
        self.profile = None
        self.t_start_ns = 0

    # ------------------------------------------------------------------ bag
    def _start_bag(self) -> bool:
        topics = list(self.get_parameter("bag_topics").value)
        out = self.run_dir / "bag"
        cmd = ["ros2", "bag", "record", "-s", str(self.get_parameter("bag_storage").value),
               "-o", str(out), "--topics", *topics]
        try:
            self.bag_proc = subprocess.Popen(cmd, stdout=open(self.run_dir / "bag_record.log", "w"),
                                             stderr=subprocess.STDOUT, start_new_session=True)
        except OSError as e:
            self.get_logger().error("ros2 bag record failed: %s" % e)
            return False
        t0 = time.monotonic()
        while time.monotonic() - t0 < 10.0:
            if self.bag_proc.poll() is not None:
                return False
            if out.exists() and any(out.glob("*.mcap")):
                break
            time.sleep(0.1)
        else:
            self.get_logger().error("bag directory did not appear within 10 s")
            return False
        # The mcap file exists before the recorder has discovered every publisher: on
        # 2026-09-28 (T20260928-03) it subscribed to this node's own topics 3.2 s after
        # START and the first seconds of cmd_vel were lost. Wait until the recorder
        # has a subscription on every bag topic that currently has a publisher.
        missing = self._recorder_missing(topics, timeout=15.0)
        if missing:
            self.get_logger().error("bag recorder did not subscribe to %s" % missing)
            return False
        self.event(ExperimentEvent.BAG_STARTED, str(out))
        return True

    def _recorder_missing(self, topics, timeout: float) -> list:
        t0 = time.monotonic()
        while True:
            live = [t for t in topics if self.count_publishers(t) > 0]
            missing = [t for t in live
                       if not any("rosbag2" in i.node_name for i in self.get_subscriptions_info_by_topic(t))]
            if not missing or time.monotonic() - t0 > timeout:
                return missing
            time.sleep(0.1)

    def _stop_bag(self):
        if self.bag_proc is None:
            return
        proc, self.bag_proc = self.bag_proc, None
        _kill_bag(proc)
        self.event(ExperimentEvent.BAG_STOPPED, "rc=%s" % proc.returncode)

    # --------------------------------------------------------------- files
    # ------------------------------------------------------- reproducibility (M3)
    def _call(self, srv_type, name: str, req, timeout: float = 1.5):
        cli = self.create_client(srv_type, name, callback_group=self.srv_group)
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                return None
            fut = cli.call_async(req)
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < timeout:
                time.sleep(0.02)
            return fut.result() if fut.done() else None
        finally:
            self.destroy_client(cli)

    def _snapshot_params(self) -> dict:
        """params.yaml in the run dir: parameters of the robot nodes (in-process service calls —
        no `ros2 param dump` subprocess, whose start would disturb the Pico ground right before
        rest_pre) and the hardware component states. Returns the meta block."""
        from rcl_interfaces.srv import GetParameters, ListParameters
        from rclpy.parameter import parameter_value_to_python
        dump, absent = {}, []
        for node in PARAM_NODES:
            names = self._call(ListParameters, node + "/list_parameters", ListParameters.Request())
            if names is None:
                absent.append(node)
                continue
            rq = GetParameters.Request()
            rq.names = list(names.result.names)
            vals = self._call(GetParameters, node + "/get_parameters", rq)
            if vals is None:
                absent.append(node)
                continue
            dump[node] = {n: parameter_value_to_python(v) for n, v in zip(rq.names, vals.values)}
        hw = []
        try:
            from controller_manager_msgs.srv import ListHardwareComponents
            r = self._call(ListHardwareComponents, "/controller_manager/list_hardware_components",
                           ListHardwareComponents.Request())
            for c in (r.component if r else []):
                hw.append({"name": c.name, "plugin": c.plugin_name, "state": c.state.label,
                           "command_interfaces": [i.name for i in c.command_interfaces],
                           "state_interfaces": [i.name for i in c.state_interfaces]})
        except ImportError:
            pass
        if self.run_dir:
            with open(self.run_dir / "params.yaml", "w") as f:
                yaml.safe_dump({"nodes": dump, "absent": absent, "hardware_components": hw},
                               f, allow_unicode=True, sort_keys=False, width=200)
        self._dumped = dump
        return {"params_file": "params.yaml", "params_absent": absent, "hardware_components": hw}

    def _configure_guard(self):
        """Per-wheel guard from diff_cont's own geometry when it was readable."""
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        dc = getattr(self, "_dumped", {}).get("/diff_cont", {})
        r = float(dc.get("wheel_radius", g("wheel_radius")))
        s = float(dc.get("wheel_separation", g("wheel_separation")))
        self.guard = WheelGuard(wheel_radius=r, wheel_separation=s,
                                min_expected=float(g("stall_min_expected")),
                                still=float(g("stall_motor_rad_s")), timeout=float(g("stall_timeout_s")))
        self.meta["guard"] = {"wheel_radius": r, "wheel_separation": s, "source": "diff_cont" if dc else "runner params",
                              "min_expected": self.guard.min_expected, "still": self.guard.still,
                              "timeout_s": self.guard.timeout}

    def _write_meta(self):
        if self.run_dir:
            with open(self.run_dir / "meta.yaml", "w") as f:
                yaml.safe_dump(self.meta, f, allow_unicode=True, sort_keys=False)

    def _snapshot_versions(self) -> dict:
        try:
            from oroha_tools.versions import write_snapshot
            snap = write_snapshot(_ws_root(), self.run_dir)
            return {"workspace": snap["workspace"], "external": snap["external"],
                    "firmware_sha256": snap["firmware"]["sha256"], "calibration": snap["calibration"],
                    "uncommitted_diff_bytes": snap["_uncommitted_diff_bytes"],
                    "untracked_files": snap["_untracked_files"]}
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn("version snapshot failed: %s" % e)
            return {"error": str(e)}

    @staticmethod
    def _load_yaml_or_inline(text: str) -> dict:
        text = text.strip()
        if not text:
            return {}
        if os.path.exists(text):
            with open(text) as f:
                return yaml.safe_load(f) or {}
        return yaml.safe_load(text) or {}

    # -------------------------------------------------------------- status
    def pub_status_msg(self):
        m = ExperimentStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state = self.state
        m.run_id = self.run_id if self.state != "IDLE" else ""
        m.path = self.profile.name if self.profile else ""
        m.t_run = self.t_run() if self.state == "RUNNING" else 0.0
        m.t_total = self.profile.duration if self.profile else 0.0
        m.segment_index = self.seg_idx
        m.segment_label = (self.profile.segments[self.seg_idx].label
                           if self.profile and 0 <= self.seg_idx < len(self.profile.segments) else "")
        m.v_cmd, m.w_cmd = self.cmd_v, self.cmd_w
        m.bag_active = self.bag_proc is not None and self.bag_proc.poll() is None
        m.preflight_ok = self.preflight_ok
        m.repeat_index, m.repeats = self.repeat_index, self.repeats
        self.pub_status.publish(m)

    def _direct_stop(self):
        try:
            from oroha_tools.md_stop import stop_all
        except Exception as e:  # noqa: BLE001
            print("direct stop unavailable: %s" % e)
            return
        port = str(self.get_parameter("md_port").value)
        for attempt in range(20):                   # ~10 s while the bus comes back
            lines = []
            res = stop_all(port, retries=1, log=lines.append)
            if all(v == "ok" for v in res.values()):
                self.event(ExperimentEvent.NOTE, "direct MD400 stop ok (attempt %d)" % (attempt + 1))
                return
            time.sleep(0.5)
        self.event(ExperimentEvent.NOTE, "direct MD400 stop FAILED after 20 attempts — use the E-stop")

    def _orphan_bags(self) -> list:
        """ros2 bag record processes writing under runs_dir that this node does not own."""
        mine = self.bag_proc.pid if self.bag_proc else None
        out = []
        for name in os.listdir("/proc"):
            if not name.isdigit() or int(name) == mine:
                continue
            try:
                cmd = open("/proc/%s/cmdline" % name, "rb").read().replace(b"\0", b" ").decode(errors="replace")
            except OSError:
                continue
            if "bag record" in cmd and str(self.runs_dir) in cmd:
                out.append("orphan bag recorder pid %s (%s) — stop it with kill -INT %s" % (name, cmd[:80], name))
        return out

    def shutdown_cleanup(self, why: str = "node shutdown"):
        """Idempotent; safe after the ROS context is gone. Order: zeros -> ABORT -> bag -> meta."""
        with self.lock:
            if self.state in ("ARMED", "RUNNING"):
                for _ in range(5):
                    self.publish_cmd(0.0, 0.0)
                    time.sleep(0.05)
                self.event(ExperimentEvent.ABORT, why, self.seg_idx)
                self._finalize("ABORTED", why)
            elif self.state in ("DONE", "ABORTED", "FAILED"):
                self._finalize(self.state, why)
            elif self.bag_proc is not None:
                self._stop_bag()

    def destroy_node(self):
        try:
            self.shutdown_cleanup()
        finally:
            try:
                super().destroy_node()
            except Exception:  # noqa: BLE001
                pass


def _kill_bag(proc) -> None:
    """SIGINT the recorder's process group and wait, so metadata.yaml gets written."""
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=15.0)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except (ProcessLookupError, OSError):
        pass


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # Own the signals (plan review H2): with rclpy's default handlers Ctrl-C shuts the
    # context down first, publishing zeros/events then fails and the bag recorder (own
    # session) is orphaned without metadata.yaml.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = ExperimentRunner()
    atexit.register(lambda: _kill_bag(node.bag_proc))
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C of a process group arrives twice (directly + via the `ros2 run` wrapper /
        # launch): ignore further signals so zero cmd -> bag stop -> meta finalize completes
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            node.shutdown_cleanup("runner stopped (SIGINT/SIGTERM)")
        finally:
            ex.shutdown(timeout_sec=1.0)
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
