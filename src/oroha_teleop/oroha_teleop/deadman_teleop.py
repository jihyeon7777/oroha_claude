"""Dead-man keyboard teleop (hold-to-drive) for the OROHA base.

  ros2 run oroha_teleop deadman_teleop                       # -> /diff_cont/cmd_vel (TwistStamped, 20 Hz)
  ros2 run oroha_teleop deadman_teleop --ros-args -p v_max:=0.3 -p w_max:=1.0

Keys (terminal auto-repeat is the dead-man signal):
  w / s   forward / backward      a / d   turn left (ccw) / right
  q / e   forward+left / forward+right          k   keepalive (hold current command)
  space or Esc   STOP (zeros immediately; diff_cont still ramps at 0.3 m/s^2)     + / -   speed scale
  x or Ctrl-C    quit (zeros sent)

Ported from the bringup campaign's test/load_manual.py: a key sets the target; while keys keep
arriving the target holds. Release grace is two-stage — `hold_arm` (0.8 s) until auto-repeat has
been observed (two keys within hold_arm), then `release_stop` (0.1 s). When the grace expires the
target ramps to zero at `decel`; space/Esc bypass the ramp. Measured basis: terminal repeat first
delay ~500 ms, period ~30 ms, so 0.8 s leaves margin. diff_cont's cmd_vel_timeout (0.5 s) is the
second line of defence; the physical E-stop the last.
"""

from __future__ import annotations

import select
import signal
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

KEYMAP = {  # key -> (v_dir, w_dir)
    # q/e at half the turn rate: radius v/w = 0.6 m instead of 0.3 m, so the inner side keeps
    # ~45 % of the outer speed (at 0.3 m the inner wheels nearly stop and are dragged ahead of
    # their command -> MD400 zero output / OVER_LOAD, T20261006-07/09)
    "w": (1, 0), "s": (-1, 0), "a": (0, 1), "d": (0, -1), "q": (1, 0.5), "e": (1, -0.5),
}


class DeadmanTeleop(Node):
    def __init__(self):
        super().__init__("oroha_teleop")
        d = self.declare_parameter
        d("cmd_topic", "/diff_cont/cmd_vel")
        d("frame_id", "base_link")
        d("rate", 20.0)
        d("v_max", 0.3)
        d("w_max", 1.0)
        d("accel", 0.3)          # m/s^2   (D-16: gentle ramps, same as diff_cont limits)
        d("decel", 0.3)          # m/s^2 on release (ramped stop)
        d("ang_accel", 1.0)      # rad/s^2
        d("hold_arm", 0.8)       # s grace before auto-repeat is observed
        d("release_stop", 0.1)   # s grace once auto-repeat is armed
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.rate = float(g("rate"))
        self.v_max, self.w_max = float(g("v_max")), float(g("w_max"))
        self.accel, self.decel, self.ang_accel = float(g("accel")), float(g("decel")), float(g("ang_accel"))
        self.hold_arm, self.release_stop = float(g("hold_arm")), float(g("release_stop"))
        self.frame_id = str(g("frame_id"))
        self.pub = self.create_publisher(TwistStamped, str(g("cmd_topic")),
                                         # RELIABLE: compatible with any diff_cont subscription QoS
                                         QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.scale = 0.5
        self.target = (0.0, 0.0)
        self.cmd = [0.0, 0.0]
        self.last_key_t = 0.0
        self.prev_key_t = 0.0
        self.hold_armed = False
        self.hard_stop = False
        self.stopped_reason = ""

    def on_key(self, k: str, now: float):
        if k in (" ", "\x1b"):
            self.hard_stop = True
            self.target = (0.0, 0.0)
            self.cmd = [0.0, 0.0]
            self.stopped_reason = "hard stop"
            return
        if k == "+" or k == "=":
            self.scale = min(1.0, self.scale + 0.1)
        elif k == "-":
            self.scale = max(0.1, self.scale - 0.1)
        if k in KEYMAP or k == "k":
            if self.hard_stop and k != "k":
                self.hard_stop = False      # a new drive key releases the latch
            if k in KEYMAP:
                vd, wd = KEYMAP[k]
                self.target = (vd * self.v_max * self.scale, wd * self.w_max * self.scale)
            # auto-repeat detection: two keys within hold_arm -> tighten the grace
            if now - self.last_key_t < self.hold_arm and self.last_key_t > 0:
                self.hold_armed = True
            self.prev_key_t, self.last_key_t = self.last_key_t, now
            self.stopped_reason = ""

    def step(self, now: float, dt: float):
        grace = self.release_stop if self.hold_armed else self.hold_arm
        if self.hard_stop:
            self.cmd = [0.0, 0.0]
        else:
            if self.last_key_t and now - self.last_key_t > grace and self.target != (0.0, 0.0):
                # /rosout evidence for the release latency (last key -> ramp start)
                self.get_logger().info("released: ramp to 0 starts %.3f s after the last key (%s)"
                                       % (now - self.last_key_t, "armed" if self.hold_armed else "single press"))
                self.target = (0.0, 0.0)
                self.hold_armed = False
                self.last_key_t = 0.0      # next press starts un-armed (0.8 s grace) — no stutter
                self.stopped_reason = "released"
            for i, (tgt, up, down) in enumerate(((self.target[0], self.accel, self.decel),
                                                 (self.target[1], self.ang_accel, self.ang_accel))):
                cur = self.cmd[i]
                lim = (down if abs(tgt) < abs(cur) or tgt * cur < 0 else up) * dt
                delta = max(-lim, min(lim, tgt - cur))
                self.cmd[i] = cur + delta
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.twist.linear.x = float(self.cmd[0])
        m.twist.angular.z = float(self.cmd[1])
        self.pub.publish(m)

    def status(self) -> str:
        return ("v %+.2f w %+.2f | target %+.2f %+.2f | scale %.1f | %s%s" % (
            self.cmd[0], self.cmd[1], self.target[0], self.target[1], self.scale,
            "HARD STOP " if self.hard_stop else ("armed " if self.hold_armed else ""), self.stopped_reason))


def main(args=None):
    if not sys.stdin.isatty():
        print("deadman_teleop needs a terminal (run it with ros2 run in a real shell)")
        return 2
    # own SIGINT: with rclpy's handler the context dies before the zeros are published
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    node = DeadmanTeleop()
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print(__doc__.split("Keys")[1].split("Ported")[0])
    period = 1.0 / node.rate
    t_pub = time.monotonic()
    try:
        while rclpy.ok():
            now = time.monotonic()
            r, _, _ = select.select([sys.stdin], [], [], 0.005)
            if r:
                k = sys.stdin.read(1)
                if k in ("x", "\x03"):
                    break
                node.on_key(k, now)
            if now - t_pub >= period:
                node.step(now, now - t_pub)
                t_pub = now
                sys.stdout.write("\r" + node.status() + " " * 8)
                sys.stdout.flush()
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)      # a second Ctrl-C must not skip the zeros
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)     # terminal first, whatever happens next
        sent = 0
        for _ in range(5):
            node.cmd = [0.0, 0.0]
            node.hard_stop = True
            try:
                node.step(time.monotonic(), period)
                sent += 1
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.05)
        print("\nstopped (%d zero commands sent; diff_cont also times out after 0.5 s)" % sent)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
