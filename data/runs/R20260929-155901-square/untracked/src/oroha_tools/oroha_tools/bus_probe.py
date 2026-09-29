"""RS485 bus probe without ros2_control: poll both MD400 monitors and log every transaction.

  oroha_bus_probe --mode idle  --sec 120 --out records/tests/<ID>/probe_idle.csv
  oroha_bus_probe --mode steps --sec 120 --out records/tests/<ID>/probe_steps.csv   # WHEELS SPIN (lifted only)

--on-lock diag: when one id fails 5 polls in a row, run a fixed diagnosis and end:
  hold 2 s (unchanged) -> solo 3 s (only the locked id is read; no traffic to the other id,
  which keeps its last command) -> the OTHER id VEL 0 for 3 s -> the locked id: stop() -> poll 3 s.
  Each row carries its phase so the recovery point can be attributed.

steps: repeats square-like transitions on both wheels (ramp up 0.2 s -> hold -> ramp down -> dwell),
alternating "forward" (id1 +rpm, id2 -rpm) and "turn" (both +rpm), at --rpm (500 ~ 0.2 m/s).
Every poll is logged (t, id, ok, latency, error, rpm, current, position, status). Ends by itself
(--sec, capped at 600 s); SIGINT/SIGTERM and exit always send VEL 0 + stop + torque_off to both ids.
The port must be free (run it before the launch).
"""
from __future__ import annotations

import argparse
import csv
import signal
import sys
import time

MD_PORT = "/dev/oroha_md400"
SEC_CAP = 600.0


class _Stop(Exception):
    pass


def _raise_stop(*_):
    raise _Stop()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=("idle", "steps"), required=True)
    ap.add_argument("--sec", type=float, default=120.0)
    ap.add_argument("--rpm", type=int, default=500)
    ap.add_argument("--hold", type=float, default=4.0, help="seconds at speed per step")
    ap.add_argument("--dwell", type=float, default=1.0, help="seconds stopped between steps")
    ap.add_argument("--period", type=float, default=0.05, help="poll period per id pair")
    ap.add_argument("--resend", action="store_true",
                    help="write VEL every poll like the ros2_control plugin (else only on change)")
    ap.add_argument("--on-lock", choices=("none", "diag"), default="none")
    ap.add_argument("--ifd", type=float, default=None,
                    help="inter-frame silence before each request, s (default: Modbus t3.5 = 2.0 ms at 19200)")
    ap.add_argument("--timeout", type=float, default=0.1)
    ap.add_argument("--port", default=MD_PORT)
    ap.add_argument("--out", required=True)
    ap.add_argument("--yes", action="store_true", help="required for --mode steps (wheels spin)")
    a = ap.parse_args(argv)
    if a.mode == "steps" and not a.yes:
        sys.exit("--mode steps spins both wheels: lift them, then pass --yes")

    from oroha_power.port_guard import require_free
    require_free(a.port, "oroha_bus_probe")

    from mdrobot import ModbusClient, SerialTransport, SingleMotorDriver
    from mdrobot import registers as reg

    tr = SerialTransport(a.port, 19200, timeout=a.timeout, inter_frame_delay=a.ifd)
    print(f"inter-frame delay {tr.inter_frame_delay * 1e3:.2f} ms, timeout {a.timeout * 1e3:.0f} ms")
    drv = {sid: SingleMotorDriver(ModbusClient(tr, slave_id=sid)) for sid in (1, 2)}
    signal.signal(signal.SIGINT, _raise_stop)
    signal.signal(signal.SIGTERM, _raise_stop)
    f = open(a.out, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "phase", "id", "op", "ok", "latency_ms", "error", "cmd_rpm", "speed_rpm", "current_a",
                "position", "status", "status2"])
    t0 = time.monotonic()
    t0_ros_ns = time.time_ns()     # wall/ROS time of t=0, to align with bags (Pico /oroha_power/sample)
    with open(a.out + ".meta", "w") as fm:
        fm.write(f"t0_ros_ns: {t0_ros_ns}\nargs: {vars(a)}\ninter_frame_delay_s: {tr.inter_frame_delay}\n")
    n_ok = {1: 0, 2: 0}
    n_fail = {1: 0, 2: 0}
    fail_t = []
    phase = ["run"]

    def txn(sid, op, fn, cmd=""):
        ts = time.monotonic()
        try:
            r = fn()
            ok, err = 1, ""
        except Exception as e:     # timeout / CRC / short read
            r, ok, err = None, 0, f"{type(e).__name__}: {e}"
        lat = (time.monotonic() - ts) * 1e3
        row = [f"{ts - t0:.4f}", phase[0], sid, op, ok, f"{lat:.1f}", err, cmd]
        if ok and op == "mon":
            row += [r.speed_rpm, r.current_a, r.position, r.status.raw, r.status2_raw]
        w.writerow(row)
        (n_ok if ok else n_fail)[sid] += 1
        if not ok:
            fail_t.append(ts - t0)
        return ok, r

    def target(t):
        """(rpm_id1, rpm_id2) for run time t."""
        if a.mode == "idle":
            return 0, 0
        cyc = a.hold + 0.4 + a.dwell
        k, u = divmod(t, cyc)
        if u < 0.2:
            s = u / 0.2
        elif u < 0.2 + a.hold:
            s = 1.0
        elif u < 0.4 + a.hold:
            s = 1.0 - (u - 0.2 - a.hold) / 0.2
        else:
            s = 0.0
        r = int(round(a.rpm * s))
        return (r, -r) if int(k) % 2 == 0 else (r, r)

    try:
        for sid, d in drv.items():
            _, v = txn(sid, "ver", d.get_version)
            _, lsw = txn(sid, "lsw", lambda d=d: d.client.read_register(reg.PID_USE_LIMIT_SW))
            print(f"id {sid}: fw v{v}  USE_LIMIT_SW {lsw}")
            if a.mode == "steps":
                txn(sid, "enable", d.enable)
        last = {1: None, 2: None}
        streak = {1: 0, 2: 0}
        hold = {}                   # id -> rpm frozen during diagnosis
        locked = None
        t_diag = None
        while (t := time.monotonic() - t0) < min(a.sec, SEC_CAP):
            tick = time.monotonic()
            cmd = dict(zip((1, 2), target(t)))
            cmd.update(hold)
            ids = (locked,) if phase[0] == "solo" else (1, 2)
            for sid in ids:              # plugin cycle order: read both, then write both
                ok, _ = txn(sid, "mon", drv[sid].read_monitor, cmd[sid])
                streak[sid] = 0 if ok else streak[sid] + 1
            for sid in ids if phase[0] != "solo" else ():
                d = drv[sid]
                if a.mode == "steps" and (a.resend or cmd[sid] != last[sid]):
                    txn(sid, "vel", lambda d=d, c=cmd[sid]: d.set_velocity(c), cmd[sid])
                    last[sid] = cmd[sid]
            if a.on_lock == "diag":
                if locked is None:
                    bad = [i for i in (1, 2) if streak[i] >= 5]
                    if bad:
                        locked = bad[0]
                        other = 3 - locked
                        t_diag = t
                        hold = {locked: cmd[locked], other: cmd[other]}
                        phase[0] = "lock_hold"
                        print(f"id {locked} locked at t={t:.2f} (other id {other} at {cmd[other]} rpm)")
                else:
                    u = t - t_diag
                    if phase[0] == "lock_hold" and u >= 2.0:
                        phase[0] = "solo"
                    elif phase[0] == "solo" and u >= 5.0:
                        phase[0] = "other_vel0"
                        hold[other] = 0
                    elif phase[0] == "other_vel0" and u >= 8.0:
                        phase[0] = "locked_stop"
                        txn(locked, "stop", drv[locked].stop)
                    elif phase[0] == "locked_stop" and u >= 11.0:
                        break
            time.sleep(max(0.0, a.period - (time.monotonic() - tick)))
    except _Stop:
        print("stopped by signal")
    finally:
        phase[0] = "end"
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        for _ in range(3):
            okall = True
            for sid, d in drv.items():
                for op, fn in (("vel0", lambda d=d: d.set_velocity(0)), ("stop", d.stop),
                               ("toff", d.torque_off)):
                    if not txn(sid, op, fn)[0] and op != "vel0":
                        okall = False
            if okall:
                break
            time.sleep(0.3)
        f.close()
        tr.close()
    dur = time.monotonic() - t0
    print(f"{dur:.1f} s  ok id1={n_ok[1]} id2={n_ok[2]}  fail id1={n_fail[1]} id2={n_fail[2]}")
    if fail_t:
        print("first failures at t =", ", ".join(f"{x:.2f}" for x in fail_t[:10]))
    print(f"log: {a.out}")


if __name__ == "__main__":
    main()
