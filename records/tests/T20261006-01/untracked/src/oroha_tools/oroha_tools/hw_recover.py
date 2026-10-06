"""Bring the MD400 hardware component back after a comm loss, without restarting the launch (M7).

  oroha_hw_recover [--component oroha_base] [--controllers joint_state_broadcaster diff_cont] [--dry-run]

When the plugin fails `max_comm_errors` reads/writes in a row (patch 0002 retries each once),
controller_manager puts the component through on_error and deactivates the controllers that
use it; diff_cont stops, but the MD400s KEEP their last command (no comm watchdog). So:

  1. E-stop pressed and held (the wheels may still be turning on the last command).
  2. Fix the cause (RS485 plug, power).
  3. oroha_hw_recover  -> component configured + activated (on_configure re-opens the port and
     writes use_limit_sw; on_activate enables with VEL 0), then the controllers are re-activated.
     diff_cont sends 0 until a new command arrives.
  4. Check /joint_states is fresh and the wheels read 0, then release the E-stop.

Every step is a controller_manager service call (no `ros2 control` subprocesses). Exit 0 only
when the component is active, the controllers are active and /joint_states arrives again.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node


def _call(node: Node, srv_type, name: str, req, timeout: float = 5.0):
    cli = node.create_client(srv_type, name)
    try:
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        return fut.result() if fut.done() else None
    finally:
        node.destroy_client(cli)


def components(node: Node, cm: str) -> dict:
    from controller_manager_msgs.srv import ListHardwareComponents
    r = _call(node, ListHardwareComponents, cm + "/list_hardware_components", ListHardwareComponents.Request())
    return {} if r is None else {c.name: c.state.label for c in r.component}


def controllers(node: Node, cm: str) -> dict:
    from controller_manager_msgs.srv import ListControllers
    r = _call(node, ListControllers, cm + "/list_controllers", ListControllers.Request())
    return {} if r is None else {c.name: c.state for c in r.controller}


def set_component(node: Node, cm: str, name: str, state_id: int, label: str) -> bool:
    from controller_manager_msgs.srv import SetHardwareComponentState
    from lifecycle_msgs.msg import State
    rq = SetHardwareComponentState.Request()
    rq.name = name
    rq.target_state = State(id=state_id, label=label)
    r = _call(node, SetHardwareComponentState, cm + "/set_hardware_component_state", rq, timeout=10.0)
    return bool(r and r.ok)


def activate_controllers(node: Node, cm: str, names: list) -> bool:
    from controller_manager_msgs.srv import SwitchController
    rq = SwitchController.Request()
    rq.activate_controllers = names
    rq.strictness = SwitchController.Request.STRICT
    rq.activate_asap = True
    rq.timeout.sec = 5
    r = _call(node, SwitchController, cm + "/switch_controller", rq, timeout=10.0)
    return bool(r and r.ok)


def wait_joint_states(node: Node, sec: float):
    from sensor_msgs.msg import JointState
    got = []
    sub = node.create_subscription(JointState, "/joint_states", lambda m: got.append(m), 10)
    t0 = time.monotonic()
    while time.monotonic() - t0 < sec and len(got) < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return got[-1] if got else None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--component", default="oroha_base")
    ap.add_argument("--controllers", nargs="+", default=["joint_state_broadcaster", "diff_cont"])
    ap.add_argument("--cm", default="/controller_manager")
    ap.add_argument("--dry-run", action="store_true", help="only report the states")
    a = ap.parse_args(argv)
    rclpy.init()
    node = rclpy.create_node("oroha_hw_recover")
    try:
        comp = components(node, a.cm)
        if not comp:
            print(f"no answer from {a.cm} — is the launch running?")
            return 2
        before = comp.get(a.component)
        print(f"component {a.component}: {before}; controllers: {controllers(node, a.cm)}")
        if before is None:
            print(f"component {a.component} not found ({list(comp)})")
            return 2
        if a.dry_run:
            return 0 if before == "active" else 1
        if before != "active":
            print("  E-stop should be pressed: after a comm loss the MD400s keep their last command")
            # from 'unconfigured' (on_error SUCCESS) configure first; from 'inactive' activate only
            if before in ("unconfigured", "finalized"):
                ok = set_component(node, a.cm, a.component, 2, "inactive")
                print(f"  configure -> inactive: {'ok' if ok else 'FAILED'}  ({components(node, a.cm).get(a.component)})")
                if not ok:
                    return 1
            ok = set_component(node, a.cm, a.component, 3, "active")
            print(f"  activate -> active: {'ok' if ok else 'FAILED'}  ({components(node, a.cm).get(a.component)})")
            if not ok:
                return 1
        ctl = controllers(node, a.cm)
        todo = [c for c in a.controllers if ctl.get(c) != "active"]
        if todo:
            ok = activate_controllers(node, a.cm, todo)
            print(f"  activate controllers {todo}: {'ok' if ok else 'FAILED'}")
        ctl = controllers(node, a.cm)
        js = wait_joint_states(node, 3.0)
        print(f"after: component {components(node, a.cm).get(a.component)}; controllers {ctl}")
        if js is None:
            print("  /joint_states: none within 3 s")
            return 1
        print("  /joint_states: " + ", ".join(f"{n} {v:+.2f} rad/s" for n, v in zip(js.name, js.velocity)))
        good = all(ctl.get(c) == "active" for c in a.controllers)
        print("RECOVERED — check the wheels read ~0 before releasing the E-stop" if good else "NOT recovered")
        return 0 if good else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
