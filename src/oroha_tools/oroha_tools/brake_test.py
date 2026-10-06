"""Lifted braking test: does the MD400 brake an overrunning wheel? (T20261006-10 follow-up)

  oroha_brake_test --yes [--modes 1 0] [--high 1500 --low 300] [--out <csv>]   # WHEELS SPIN — lifted only

For each USE_LIMIT_SW value in --modes: enable both MD400s, run both wheels at --high rpm for
--hold s, then step the command DOWN to --low rpm (still non-zero, same direction) for --hold s,
then 0. A drive that brakes pulls the wheel down to --low quickly with negative (regenerating)
current; one that only drives lets it coast down on friction. Polls both monitors at ~20 Hz.

USE_LIMIT_SW = 0 disables the CTRL stop gates, so the E-STOP DOES NOTHING in that phase: every
phase is a few seconds and ends by itself, and USE_LIMIT_SW is written back to 1 (and read back)
at the end, on Ctrl-C/SIGTERM and on any error. Record the Pico currents in parallel (oroha_power +
ros2 bag) for the current sign; the MD400's own current reading is unsigned.
"""

from __future__ import annotations

import argparse
import csv
import signal
import sys
import time

MD_PORT = "/dev/oroha_md400"


class _Stop(Exception):
    pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--yes", action="store_true", help="wheels are lifted, E-stop in hand")
    ap.add_argument("--modes", type=int, nargs="+", default=[1, 0])
    ap.add_argument("--high", type=int, default=1500)
    ap.add_argument("--low", type=int, default=300)
    ap.add_argument("--hold", type=float, default=2.0)
    ap.add_argument("--port", default=MD_PORT)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    if not a.yes:
        sys.exit("wheels spin up to --high rpm: lift the robot, then pass --yes")
    if any(m not in (0, 1) for m in a.modes) or a.high > 2000 or not 0 < a.low < a.high:
        sys.exit("modes must be 0/1, high <= 2000, 0 < low < high")
    from oroha_power.port_guard import require_free
    require_free(a.port, "oroha_brake_test")
    from mdrobot import ModbusClient, SerialTransport, SingleMotorDriver
    from mdrobot import registers as reg

    tr = SerialTransport(a.port, 19200, timeout=0.1)
    drv = {sid: SingleMotorDriver(ModbusClient(tr, slave_id=sid)) for sid in (1, 2)}

    def _raise(*_):
        raise _Stop()
    signal.signal(signal.SIGINT, _raise)
    signal.signal(signal.SIGTERM, _raise)
    f = open(a.out, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t_wall_ns", "t", "mode", "phase", "id", "cmd_rpm", "speed_rpm", "current_a", "status"])
    t0 = time.monotonic()

    def run_phase(mode, phase, rpm, sec):
        for d in drv.values():
            d.set_velocity(rpm)
        end = time.monotonic() + sec
        while time.monotonic() < end:
            tick = time.monotonic()
            for sid, d in drv.items():
                d.set_velocity(rpm)                      # the plugin re-sends every cycle too
                m = d.read_monitor()
                w.writerow([time.time_ns(), f"{time.monotonic() - t0:.3f}", mode, phase, sid, rpm,
                            m.speed_rpm, m.current_a, m.status.raw])
            time.sleep(max(0.0, 0.05 - (time.monotonic() - tick)))

    def safe_end():
        for _ in range(3):
            ok = True
            for d in drv.values():
                try:
                    d.set_velocity(0)
                    d.client.write_register(reg.PID_USE_LIMIT_SW, 1)
                except Exception:  # noqa: BLE001
                    ok = False
            if ok:
                break
            time.sleep(0.3)
        time.sleep(0.5)
        for sid, d in drv.items():
            try:
                d.torque_off()
                print(f"id{sid}: USE_LIMIT_SW read back {d.client.read_register(reg.PID_USE_LIMIT_SW)} (must be 1), torque off")
            except Exception as e:  # noqa: BLE001
                print(f"id{sid}: final read failed: {e} — CHECK USE_LIMIT_SW BEFORE THE NEXT GROUND RUN")

    try:
        for mode in a.modes:
            for d in drv.values():
                d.client.write_register(reg.PID_USE_LIMIT_SW, mode)
                d.enable()
            got = [d.client.read_register(reg.PID_USE_LIMIT_SW) for d in drv.values()]
            print(f"USE_LIMIT_SW={mode} (read back {got}): {a.high} rpm {a.hold}s -> {a.low} rpm {a.hold}s -> 0")
            run_phase(mode, "high", a.high, a.hold)
            run_phase(mode, "low", a.low, a.hold)
            run_phase(mode, "zero", 0, 1.5)
    except _Stop:
        print("stopped by signal")
    except Exception as e:  # noqa: BLE001
        print(f"error: {type(e).__name__}: {e}")
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        safe_end()
        f.close()
        tr.close()
    # quick summary: time from the step to within 10 % of --low
    import collections
    rows = list(csv.DictReader(open(a.out)))
    res = collections.defaultdict(dict)
    for mode in a.modes:
        for sid in ("1", "2"):
            low = [r for r in rows if r["mode"] == str(mode) and r["phase"] == "low" and r["id"] == sid]
            if not low:
                continue
            t_step = float(low[0]["t"])
            settle = next((float(r["t"]) - t_step for r in low if abs(abs(int(r["speed_rpm"])) - a.low) <= 0.1 * a.low), None)
            res[mode][sid] = settle
            print(f"USE_LIMIT_SW={mode} id{sid}: {a.high}->{a.low} rpm settled in "
                  f"{'%.2f s' % settle if settle is not None else 'NOT within ' + str(a.hold) + ' s'}")
    print(f"log: {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
