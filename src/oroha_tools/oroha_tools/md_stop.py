"""Stop both MD400s now: VEL_CMD 0 -> stop -> torque off, for id 2 (LEFT) and id 1 (RIGHT).

  oroha_md_stop                       # both controllers, torque off afterwards
  oroha_md_stop --keep-torque         # zero speed only (wheels held by the speed loop)

The MD400 has no communication watchdog: when the host stops talking (launch Ctrl-C,
ros2_control_node crash, RS485 re-plug) the last velocity command stays latched. This
tool is the host-side stop. robot.launch.py runs the same stop_all() when
ros2_control_node exits. Logic power stays up during an E-stop, so this also works
while the E-stop is pressed (run it BEFORE releasing the E-stop).

It does not refuse when the port is open elsewhere (emergency use) but warns: if
ros2_control_node is still running it will keep writing its own command.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

MD_PORT = "/dev/oroha_md400"
SIDES = {2: "LEFT", 1: "RIGHT"}


def stop_all(port: str = MD_PORT, ids=(2, 1), retries: int = 3, torque_off: bool = True,
             timeout: float = 0.3, log=print) -> dict:
    """Best-effort stop of every id; never raises. Returns {id: 'ok' | 'error: ...'}."""
    from mdrobot import SingleMotorDriver

    result = {}
    for sid in ids:
        steps = ["set_velocity(0)", "stop()"] + (["torque_off()"] if torque_off else [])
        last_err = None
        for attempt in range(1, retries + 1):
            try:
                with SingleMotorDriver.open(port, slave_id=sid, timeout=timeout) as d:
                    d.set_velocity(0)
                    d.stop()
                    if torque_off:
                        d.torque_off()
                result[sid] = "ok"
                last_err = None
                break
            except Exception as e:  # noqa: BLE001 - emergency path, keep going
                last_err = f"{type(e).__name__}: {e}"
                time.sleep(0.05)
        if last_err:
            result[sid] = f"error: {last_err}"
        if log:
            log(f"id {sid} ({SIDES.get(sid, '?')}): {' + '.join(steps)} -> {result[sid]}")
    return result


def main(argv=None) -> int:
    # finish the stop even if someone sends SIGTERM (e.g. `timeout`)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--md-port", default=MD_PORT)
    ap.add_argument("--ids", default="2,1", help="slave ids, default 2,1 (left, right)")
    ap.add_argument("--retries", type=int, default=3)
    ap.add_argument("--keep-torque", action="store_true", help="do not send torque_off")
    a = ap.parse_args(argv)
    try:
        from oroha_power.port_guard import describe, port_holders
        holders = port_holders(a.md_port)
        if holders:
            print(f"WARNING: {a.md_port} is also open by {describe(holders)}")
    except Exception:  # noqa: BLE001
        pass
    ids = tuple(int(x) for x in a.ids.split(",") if x.strip())
    res = stop_all(a.md_port, ids, a.retries, torque_off=not a.keep_torque)
    ok = all(v == "ok" for v in res.values())
    print("ALL STOPPED" if ok else "NOT ALL STOPPED — use the E-stop")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
