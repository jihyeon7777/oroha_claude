"""Watch both MD400s' digital inputs (CTRL gates) and status bits; print every change. NO motion.

  oroha_di_watch --sec 60 [--out records/tests/<ID>/di_watch.csv]

Why (T20261006-07): on the ground one MD400 at a time stopped driving for seconds with status 0,
current at the quiescent level and the wheel coasting — the E-stop signature (2NC E-stop ->
CTRL pin 6 DIR + pin 8 START/STOP of EACH MD400, USE_LIMIT_SW 1). This polls PID_DI and the
monitor of id 1 (RIGHT) and id 2 (LEFT) at --rate and prints DI/status changes with times, so
pressing the E-stop halfway or wiggling its cable/connectors shows whether one side's gate opens
alone. At rest with the E-stop released DI = 0x14 (DIR, START_STOP) + encoder bits b5/b6
(ENC_B/ENC_A, which follow the wheel position and are ignored for change detection).
The port must be free (run it without the launch).
"""

from __future__ import annotations

import argparse
import csv
import signal
import sys
import time

MD_PORT = "/dev/oroha_md400"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sec", type=float, default=60.0)
    ap.add_argument("--rate", type=float, default=20.0, help="polls per second (both ids)")
    ap.add_argument("--port", default=MD_PORT)
    ap.add_argument("--out", help="CSV of every poll")
    a = ap.parse_args(argv)
    from oroha_power.port_guard import require_free
    require_free(a.port, "oroha_di_watch")
    from mdrobot import (DI_BIT_NAMES, STATUS1_BIT_NAMES, ModbusClient, SerialTransport,
                         SingleMotorDriver, active_bits)
    from mdrobot import registers as reg

    tr = SerialTransport(a.port, 19200, timeout=0.1)
    drv = {1: SingleMotorDriver(ModbusClient(tr, slave_id=1)), 2: SingleMotorDriver(ModbusClient(tr, slave_id=2))}
    side = {1: "RIGHT", 2: "LEFT"}
    stop = [False]
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    f = open(a.out, "w", newline="") if a.out else None
    w = csv.writer(f) if f else None
    if w:
        w.writerow(["t", "id", "ok", "di", "status", "status2", "rpm", "current_a"])
    last = {1: None, 2: None}
    n_change = {1: 0, 2: 0}
    t0 = time.monotonic()
    print(f"watching DI/status of id1 RIGHT and id2 LEFT for {a.sec:.0f} s (Ctrl-C ends) — rest value DI 0x34")
    while not stop[0] and time.monotonic() - t0 < a.sec:
        tick = time.monotonic()
        for sid, d in drv.items():
            t = time.monotonic() - t0
            try:
                di = d.client.read_register(reg.PID_DI)
                m = d.read_monitor()
                row = (di & 0x1F, m.status.raw, m.status2_raw)    # ENC_A/ENC_B (b5, b6) follow the wheel
                if w:
                    w.writerow([f"{t:.3f}", sid, 1, di, m.status.raw, m.status2_raw, m.speed_rpm, m.current_a])
                if row != last[sid]:
                    if last[sid] is not None:
                        n_change[sid] += 1
                    print(f"{t:8.3f} id{sid} {side[sid]:5s} DI 0x{di:04x} {active_bits(di, DI_BIT_NAMES)}  "
                          f"status 0x{m.status.raw:02x} {active_bits(m.status.raw, STATUS1_BIT_NAMES)} "
                          f"status2 0x{m.status2_raw:02x}")
                    last[sid] = row
            except Exception as e:  # noqa: BLE001
                if w:
                    w.writerow([f"{t:.3f}", sid, 0, "", "", "", "", ""])
                print(f"{t:8.3f} id{sid} read failed: {type(e).__name__}")
        time.sleep(max(0.0, 1.0 / a.rate - (time.monotonic() - tick)))
    tr.close()
    if f:
        f.close()
    print(f"changes: RIGHT {n_change[1]}  LEFT {n_change[2]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
