"""RS485 bus probe without ros2_control: poll both MD400 monitors and log every transaction.

  oroha_bus_probe --mode idle  --sec 120 --out records/tests/<ID>/probe_idle.csv
  oroha_bus_probe --mode steps --sec 120 --out records/tests/<ID>/probe_steps.csv   # WHEELS SPIN (lifted only)

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

    tr = SerialTransport(a.port, 19200, timeout=a.timeout)
    drv = {sid: SingleMotorDriver(ModbusClient(tr, slave_id=sid)) for sid in (1, 2)}
    signal.signal(signal.SIGINT, _raise_stop)
    signal.signal(signal.SIGTERM, _raise_stop)
    f = open(a.out, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "id", "op", "ok", "latency_ms", "error", "cmd_rpm", "speed_rpm", "current_a",
                "position", "status", "status2"])
    t0 = time.monotonic()
    n_ok = {1: 0, 2: 0}
    n_fail = {1: 0, 2: 0}
    fail_t = []

    def txn(sid, op, fn, cmd=""):
        ts = time.monotonic()
        try:
            r = fn()
            ok, err = 1, ""
        except Exception as e:     # timeout / CRC / short read
            r, ok, err = None, 0, f"{type(e).__name__}: {e}"
        lat = (time.monotonic() - ts) * 1e3
        row = [f"{ts - t0:.4f}", sid, op, ok, f"{lat:.1f}", err, cmd]
        if ok and op == "mon":
            row += [r.speed_rpm, r.current_a, r.position, r.status.raw, r.status2_raw]
        w.writerow(row)
        (n_ok if ok else n_fail)[sid] += 1
        if not ok:
            fail_t.append(ts - t0)
        return r

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
            v = txn(sid, "ver", d.get_version)
            lsw = txn(sid, "lsw", lambda d=d: d.client.read_register(reg.PID_USE_LIMIT_SW))
            print(f"id {sid}: fw v{v}  USE_LIMIT_SW {lsw}")
            if a.mode == "steps":
                txn(sid, "enable", d.enable)
        last = {1: None, 2: None}
        while (t := time.monotonic() - t0) < min(a.sec, SEC_CAP):
            tick = time.monotonic()
            cmd = dict(zip((1, 2), target(t)))
            for sid, d in drv.items():
                if a.mode == "steps" and cmd[sid] != last[sid]:
                    txn(sid, "vel", lambda d=d, c=cmd[sid]: d.set_velocity(c), cmd[sid])
                    last[sid] = cmd[sid]
                txn(sid, "mon", d.read_monitor, cmd[sid])
            time.sleep(max(0.0, a.period - (time.monotonic() - tick)))
    except _Stop:
        print("stopped by signal")
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        for _ in range(3):
            okall = True
            for sid, d in drv.items():
                for op, fn in (("vel0", lambda d=d: d.set_velocity(0)), ("stop", d.stop),
                               ("toff", d.torque_off)):
                    if txn(sid, op, fn) is None and op != "vel0":
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
