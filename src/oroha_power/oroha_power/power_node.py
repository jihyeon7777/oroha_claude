"""OROHA current / bus-voltage measurement node — Pico (USB CDC) -> ROS 2.

Publishes
  ~/sample             oroha_msgs/PowerSample   every firmware window (50 Hz): raw ADC + converted
  ~/battery            sensor_msgs/BatteryState 10 Hz, suppressed while the stream is stale
  ~/calibration_event  std_msgs/String          latched; '#ZERO ...' replies, calibration load
  /diagnostics         diagnostic_msgs/DiagnosticArray 1 Hz
  ~/measured ~/raw ~/power  geometry_msgs/Vector3Stamped   only with legacy_topics:=true
                       (old 1.x layout: x=GP28 right, y=GP27 left, z=V)
Services
  ~/zero               std_srvs/Trigger   send 'Z' (motors must be at rest); non-blocking for the rest of the node

Time base: one clock — the node clock. device_stamp = t_us + min-filter offset.
Channel/wheel mapping (2026-08-14): GP28 = sensor #1 = id 1 = RIGHT, GP27 = sensor #2 = id 2 = LEFT.
"""

from __future__ import annotations

import collections
import math
import os
import random
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from oroha_msgs.msg import PowerSample
from oroha_power.port_guard import describe, port_holders
from oroha_power.protocol import (FLAG_BAD_MASK, FLAG_ZERO_VALID, Calibration, Frame,
                                  OffsetFilter, convert, flag_names, parse_data_line, parse_kv)

EXPECTED_FW = "oroha-bench-1.2"


class SimulatedPico:
    """Stand-in for the serial port: emits plausible D lines at `rate` Hz (no device)."""

    def __init__(self, rate: int):
        self.rate = rate
        self.seq = 0
        self.t0 = time.monotonic()
        self.next_t = self.t0
        self.streaming = False
        self.pending = ["#CFG fw=%s smps_pwm=0 (simulated)" % EXPECTED_FW, "#READY"]
        self.is_open = True

    def write(self, b: bytes):
        c = b.decode().strip()
        if c == "S":
            self.streaming = True
            self.pending.append("#COL seq,t_us,n,v_mean,v_min,v_max,gp27_mean,gp27_min,gp27_max,gp28_mean,gp28_min,gp28_max,flags")
        elif c == "X":
            self.streaming = False
            self.pending.append("#STOP seq=%d overruns=0" % self.seq)
        elif c == "Z":
            self.pending.append("#ZERO gp28=2034.00 gp27=2035.30 rail=3.2887 rail_corr=1.0000 n=1024")
        elif c == "C":
            self.pending.append("#CFG fw=%s smps_pwm=0 (simulated)" % EXPECTED_FW)

    def flush(self):
        pass

    def readline(self) -> bytes:
        if self.pending:
            return (self.pending.pop(0) + "\n").encode()
        if not self.streaming:
            time.sleep(0.05)
            return b""
        now = time.monotonic()
        if now < self.next_t:
            time.sleep(self.next_t - now)
        self.next_t += 1.0 / self.rate
        t_us = int((time.monotonic() - self.t0) * 1e6)
        v = 3123.0 + random.gauss(0, 0.8)                      # ~28 V
        i27 = 2035.3 + 7.0 + random.gauss(0, 1.0)               # left, powered rest
        i28 = 2034.0 + 7.0 + random.gauss(0, 1.0)               # right
        line = "D,%d,%d,32,%.2f,%d,%d,%.2f,%d,%d,%.2f,%d,%d,%d" % (
            self.seq, t_us, v, int(v) - 2, int(v) + 2, i27, int(i27) - 3, int(i27) + 3,
            i28, int(i28) - 3, int(i28) + 3, FLAG_ZERO_VALID)
        self.seq += 1
        return (line + "\n").encode()

    def reset_input_buffer(self):
        pass

    def close(self):
        self.is_open = False


class OrohaPowerNode(Node):

    def __init__(self):
        super().__init__("oroha_power")
        d = self.declare_parameter
        d("port", "/dev/oroha_pico")
        d("baud", 115200)
        d("frame_id", "oroha_power")
        d("rate", 50)                   # 'P<hz>' sent on connect; 0 = firmware default
        d("auto_start", True)           # 'S' on connect
        d("zero_on_start", False)       # 'Z' on connect — motors must be at rest
        d("calibration_file", "")       # "" -> share/oroha_power/config/calibration/sensing-20260828.yaml
        d("expected_fw", EXPECTED_FW)
        d("stale_timeout", 0.5)         # s without a frame -> stale
        d("sync_window", 500)
        d("battery_rate", 10.0)
        d("diag_rate", 1.0)
        d("design_capacity", 20.0)      # Ah
        d("legacy_topics", False)
        d("simulate", False)

        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.port = str(g("port"))
        self.baud = int(g("baud"))
        self.frame_id = str(g("frame_id"))
        self.stale_timeout = float(g("stale_timeout"))
        self.simulate = bool(g("simulate"))
        self.expected_fw = str(g("expected_fw"))

        self.cal = self._load_calibration(str(g("calibration_file")))
        self.zero_gp27 = self.cal.zero_gp27
        self.zero_gp28 = self.cal.zero_gp28
        self.rail_corr = 1.0
        self.fw_version = ""

        # publishers
        sensor_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_sample = self.create_publisher(PowerSample, "~/sample", sensor_qos)
        self.pub_bat = self.create_publisher(BatteryState, "~/battery", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.pub_event = self.create_publisher(String, "~/calibration_event", latched)
        self.legacy = bool(g("legacy_topics"))
        if self.legacy:
            self.pub_meas = self.create_publisher(Vector3Stamped, "~/measured", sensor_qos)
            self.pub_raw = self.create_publisher(Vector3Stamped, "~/raw", sensor_qos)
            self.pub_pow = self.create_publisher(Vector3Stamped, "~/power", sensor_qos)

        # state
        self.q: collections.deque = collections.deque(maxlen=2000)
        self.lock = threading.Lock()
        self.offset = OffsetFilter(int(g("sync_window")))
        self.ser = None
        self.running = True
        self.n_frames = 0
        self.n_gaps = 0
        self.n_bad = 0
        self.n_overrun = 0
        self.last_seq = None
        self.flags_acc = 0
        self.last: dict | None = None
        self.last_rx_ns = 0
        self.stale = True
        self.t_open = time.monotonic()
        self.zero_event = threading.Event()
        self.zero_reply = ""

        self._event("calibration_loaded calib_id=%s" % self.cal.calib_id)

        # service in its own reentrant group so the 3 s wait never blocks timers
        self.srv_group = ReentrantCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, "~/zero", self.on_zero, callback_group=self.srv_group)

        self.open_serial()
        self.rd_thread = threading.Thread(target=self.reader, daemon=True)
        self.rd_thread.start()

        self.create_timer(0.01, self.drain, callback_group=self.timer_group)               # 100 Hz
        self.create_timer(1.0 / float(g("battery_rate")), self.pub_battery, callback_group=self.timer_group)
        self.create_timer(1.0 / float(g("diag_rate")), self.pub_diagnostics, callback_group=self.timer_group)
        self.create_timer(0.2, self.check_stale, callback_group=self.timer_group)

        self.get_logger().info("oroha_power: %s @ %d, calib %s%s" % (
            self.port, self.baud, self.cal.calib_id, " (SIMULATED)" if self.simulate else ""))

    # ------------------------------------------------------------ calibration
    def _load_calibration(self, path: str) -> Calibration:
        if not path:
            path = os.path.join(get_package_share_directory("oroha_power"),
                                "config", "calibration", "sensing-20260828.yaml")
        with open(path) as f:
            cal = Calibration.from_dict(yaml.safe_load(f))
        self.calibration_path = path
        return cal

    def _event(self, text: str):
        m = String()
        m.data = "%d %s" % (self.get_clock().now().nanoseconds, text)
        self.pub_event.publish(m)

    # ----------------------------------------------------------------- serial
    def open_serial(self):
        try:
            if self.simulate:
                self.ser = SimulatedPico(int(self.get_parameter("rate").value) or 50)
            else:
                import serial
                holders = port_holders(self.port)
                if holders:
                    # a preflight/bench tool is talking to the Pico; opening now would
                    # interleave commands (X/Z/S) — retry in the reader loop
                    self.get_logger().error("%s is open by %s — not opening (retry in 1 s)"
                                            % (self.port, describe(holders)))
                    self.ser = None
                    return
                self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
                time.sleep(0.3)
                self.ser.reset_input_buffer()
            self.send("C")
            rate = int(self.get_parameter("rate").value)
            if rate:
                self.send("P%d" % rate)     # 50 Hz is the firmware ceiling; 100 -> permanent overrun
                time.sleep(0.2)
            if self.get_parameter("zero_on_start").value:
                self.send("Z")
                time.sleep(1.5)
            if self.get_parameter("auto_start").value:
                self.send("S")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error("open %s failed: %s" % (self.port, e))
            self.ser = None

    def send(self, s: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((s + "\n").encode())
                self.ser.flush()
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn("write failed: %s" % e)

    def reader(self):
        while self.running:
            if not self.ser or not self.ser.is_open:
                time.sleep(1.0)
                self.open_serial()
                continue
            try:
                raw = self.ser.readline()
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn("read failed, reconnecting: %s" % e)
                try:
                    self.ser.close()
                except Exception:  # noqa: BLE001
                    pass
                self.ser = None
                continue
            if not raw:
                continue
            rx_ns = self.get_clock().now().nanoseconds
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            if line.startswith("#"):
                self.on_meta(line)
                continue
            if line.startswith("D,"):
                f = parse_data_line(line)
                if f is None:
                    self.n_bad += 1
                    continue
                with self.lock:
                    self.q.append((rx_ns, f))

    def on_meta(self, line: str):
        if line.startswith("#ZERO"):
            kv = parse_kv(line)
            try:
                if "gp27" in kv:
                    self.zero_gp27 = float(kv["gp27"])
                if "gp28" in kv:
                    self.zero_gp28 = float(kv["gp28"])
                if "rail_corr" in kv:
                    self.rail_corr = float(kv["rail_corr"])
            except ValueError:
                pass
            self.get_logger().info(line)
            self._event(line)
            self.zero_reply = line
            self.zero_event.set()
        elif line.startswith("#CFG"):
            kv = parse_kv(line)
            if "fw" in kv:
                self.fw_version = kv["fw"]
                if self.fw_version != self.expected_fw:
                    self.get_logger().warn("firmware %s, expected %s" % (self.fw_version, self.expected_fw))
            self.get_logger().debug(line)
        elif line.startswith("#ERR") or line.startswith("#WARN"):
            self.get_logger().warn(line)

    # ---------------------------------------------------------------- publish
    def drain(self):
        with self.lock:
            batch = list(self.q)
            self.q.clear()
        for rx_ns, f in batch:
            try:
                self.publish(rx_ns, f)
            except (OverflowError, AssertionError, ValueError) as e:
                # one malformed frame must not take the node down; count it and go on
                self.n_bad += 1
                if self.n_bad <= 5 or self.n_bad % 100 == 0:
                    self.get_logger().warn("frame seq %d rejected (%s): %s" % (f.seq, type(e).__name__, e))

    def publish(self, rx_ns: int, f: Frame):
        if self.last_seq is not None and f.seq > self.last_seq + 1:
            self.n_gaps += f.seq - self.last_seq - 1
        self.last_seq = f.seq
        self.n_frames += 1
        self.flags_acc |= f.flags
        if f.overrun:
            self.n_overrun += 1
        self.last_rx_ns = rx_ns
        self.stale = False

        off_s, resid_s = self.offset.update(rx_ns * 1e-9, f.t_us * 1e-6)
        dev_ns = int(round((f.t_us * 1e-6 + off_s) * 1e9))
        v, i_left, i_right = convert(f, self.cal, self.zero_gp27, self.zero_gp28, self.rail_corr)

        m = PowerSample()
        m.header.stamp = Time(nanoseconds=rx_ns).to_msg()
        m.header.frame_id = self.frame_id
        m.device_stamp = Time(nanoseconds=dev_ns).to_msg()
        m.seq, m.t_us, m.n_rounds = max(0, f.seq), f.t_us, max(0, min(65535, f.n))
        m.gp26_mean, m.gp26_min, m.gp26_max = f.v, f.v_lo, f.v_hi
        m.gp27_mean, m.gp27_min, m.gp27_max = f.gp27, f.gp27_lo, f.gp27_hi
        m.gp28_mean, m.gp28_min, m.gp28_max = f.gp28, f.gp28_lo, f.gp28_hi
        m.flags = f.flags
        m.zero_valid = f.zero_valid
        m.overrun = f.overrun
        m.v_bus, m.i_left, m.i_right = v, i_left, i_right
        m.p_left, m.p_right = v * i_left, v * i_right
        m.p_total = m.p_left + m.p_right
        m.zero_gp27, m.zero_gp28, m.rail_corr = self.zero_gp27, self.zero_gp28, self.rail_corr
        m.sync_offset_s = off_s
        m.sync_residual_ms = resid_s * 1e3
        m.calib_id = self.cal.calib_id
        m.fw_version = self.fw_version
        self.pub_sample.publish(m)

        if self.legacy:
            hdr = m.header
            a = Vector3Stamped(); a.header = hdr
            a.vector.x, a.vector.y, a.vector.z = i_right, i_left, v
            self.pub_meas.publish(a)
            r = Vector3Stamped(); r.header = hdr
            r.vector.x, r.vector.y, r.vector.z = f.gp28, f.gp27, f.v
            self.pub_raw.publish(r)
            w = Vector3Stamped(); w.header = hdr
            w.vector.x, w.vector.y, w.vector.z = v * i_right, v * i_left, m.p_total
            self.pub_pow.publish(w)

        self.last = dict(v=v, i_left=i_left, i_right=i_right, resid_ms=resid_s * 1e3, f=f)

    def check_stale(self):
        if self.last_rx_ns and not self.stale:
            age = (self.get_clock().now().nanoseconds - self.last_rx_ns) * 1e-9
            if age > self.stale_timeout:
                self.stale = True
                self.get_logger().warn("stream stale: no frame for %.2f s" % age)

    def pub_battery(self):
        if not self.last or self.stale:
            return
        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.header.frame_id = self.frame_id
        b.voltage = float(self.last["v"])
        b.current = float(-(self.last["i_left"] + self.last["i_right"]))   # ROS: discharge negative
        b.charge = b.capacity = b.percentage = float("nan")
        b.design_capacity = float(self.get_parameter("design_capacity").value)
        b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        b.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        b.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        b.present = True
        b.location = "motor_bus"
        self.pub_bat.publish(b)

    def pub_diagnostics(self):
        st = DiagnosticStatus()
        st.name = "oroha_power: Pico current/voltage"
        st.hardware_id = self.port
        kv = st.values.append
        names = flag_names(self.flags_acc)
        bad = [n for n in names if n != "zero_valid"]

        if self.last is None:
            st.level = DiagnosticStatus.ERROR if self.ser is None else DiagnosticStatus.WARN
            st.message = "no frames yet" if self.ser else "port not open"
        elif self.stale:
            st.level = DiagnosticStatus.ERROR
            st.message = "stale: no frame for > %.1f s" % self.stale_timeout
        elif self.fw_version and self.fw_version != self.expected_fw:
            st.level = DiagnosticStatus.WARN
            st.message = "firmware %s != expected %s" % (self.fw_version, self.expected_fw)
        elif not (self.flags_acc & FLAG_ZERO_VALID):
            st.level = DiagnosticStatus.WARN
            st.message = "zero not valid — call ~/zero with motors at rest"
        elif bad:
            st.level = DiagnosticStatus.WARN
            st.message = "flags: " + ", ".join(bad)
        else:
            st.level = DiagnosticStatus.OK
            st.message = "OK"

        up = max(1e-6, time.monotonic() - self.t_open)
        kv(KeyValue(key="rate_hz", value="%.2f" % (self.n_frames / up)))
        kv(KeyValue(key="frames", value=str(self.n_frames)))
        kv(KeyValue(key="seq_gaps", value=str(self.n_gaps)))
        kv(KeyValue(key="parse_errors", value=str(self.n_bad)))
        kv(KeyValue(key="overrun_pct", value="%.2f" % (100.0 * self.n_overrun / max(1, self.n_frames))))
        kv(KeyValue(key="device_restarts", value=str(self.offset.restarts)))
        kv(KeyValue(key="sync_window", value=str(len(self.offset))))
        kv(KeyValue(key="fw_version", value=self.fw_version or "?"))
        kv(KeyValue(key="calib_id", value=self.cal.calib_id))
        kv(KeyValue(key="zero_gp28/gp27", value="%.2f / %.2f" % (self.zero_gp28, self.zero_gp27)))
        kv(KeyValue(key="rail_corr", value="%.6f" % self.rail_corr))
        if self.last:
            f = self.last["f"]
            kv(KeyValue(key="sync_residual_ms", value="%.2f" % self.last["resid_ms"]))
            kv(KeyValue(key="V_bus", value="%.4f V" % self.last["v"]))
            kv(KeyValue(key="I_right(GP28,id1)", value="%+.4f A" % self.last["i_right"]))
            kv(KeyValue(key="I_left(GP27,id2)", value="%+.4f A" % self.last["i_left"]))
            kv(KeyValue(key="P_total", value="%.3f W" % (self.last["v"] * (self.last["i_left"] + self.last["i_right"]))))
            kv(KeyValue(key="raw_V/GP27/GP28", value="%.1f / %.1f / %.1f" % (f.v, f.gp27, f.gp28)))
            kv(KeyValue(key="flags_seen", value=",".join(names) if names else "-"))

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(st)
        self.pub_diag.publish(arr)
        self.flags_acc = 0

    # ---------------------------------------------------------------- service
    def on_zero(self, req, res):
        self.zero_event.clear()
        self.zero_reply = ""
        self.send("Z")
        if self.zero_event.wait(3.0):
            res.success = True
            res.message = self.zero_reply
        else:
            res.success = False
            res.message = "no #ZERO reply within 3 s"
        return res

    def destroy_node(self):
        self.running = False
        try:
            self.send("X")
            time.sleep(0.1)
            if self.ser:
                self.ser.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    import signal
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = OrohaPowerNode()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # a terminal/process-group Ctrl-C reaches us twice (directly + forwarded by launch):
        # the second must not abort the cleanup (T20261001-01)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
