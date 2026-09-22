"""Pre-run system check — the motors do NOT turn.

Run BEFORE robot.launch.py (it opens the RS485 port itself):

  oroha_preflight                     # 15 s of Pico rest data
  oroha_preflight --sec 30 --no-zero  # keep the firmware zero, longer statistics

Checks (read-only, no enable(), no velocity command, no config-register writes):
  1. ports     — device nodes exist and are writable
  2. MD400     — id 1 / id 2 answer, firmware version, bus voltage + gap, status alarms, rpm == 0
  3. Pico      — firmware version, sample interval, seq gaps, overrun, rail range, zero_valid
  4. rest raw  — per-channel rest raw vs the 2026-08-28 reference band; bus voltage from GP26

Result: console go/no-go and records/preflight/<UTC timestamp>.json (exit 1 when a
blocking check fails). Ported from the bringup campaign's test/preflight.py.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
from pathlib import Path

import yaml

from oroha_tools.ws import records_dir

MD_PORT = "/dev/oroha_md400"
PICO_PORT = "/dev/oroha_pico"
EXPECTED_FW = "oroha-bench-1.2"
VOLT_GAP = 0.600       # id2 - id1 internal voltmeter offset, reproduced over 6 sessions [V]
# powered-rest raw band, 2026-08-28 (four observations); true 0 A drifts +-2 LSB/day
REST_REF = {"gp27": (2042.2, 2043.3), "gp28": (2039.6, 2040.7)}
REST_TOL = 8.0         # LSB ~ 0.1 A


def _calibration() -> dict:
    """Current/voltage constants from the installed calibration YAML."""
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory("oroha_power"), "config", "calibration",
                        "sensing-20260828.yaml")
    with open(path) as f:
        return yaml.safe_load(f)


class Report:
    def __init__(self):
        self.checks: list = []

    def ok(self, cond: bool, msg: str, blocking: bool = True) -> bool:
        self.checks.append({"ok": bool(cond), "msg": msg, "blocking": blocking})
        tag = "OK  " if cond else ("FAIL" if blocking else "WARN")
        print(f"    {tag}  {msg}")
        return bool(cond)

    @property
    def failures(self):
        return [c["msg"] for c in self.checks if not c["ok"] and c["blocking"]]

    @property
    def warnings(self):
        return [c["msg"] for c in self.checks if not c["ok"] and not c["blocking"]]


def check_ports(r: Report, md_port: str, pico_port: str) -> None:
    print("\n[1] ports")
    for name, path in (("MD400", md_port), ("Pico", pico_port)):
        if not Path(path).exists():
            r.ok(False, f"{name}: {path} missing — USB / udev (setup/udev/99-oroha.rules)")
            continue
        real = os.path.realpath(path)
        writable = os.access(real, os.R_OK | os.W_OK)
        r.ok(writable, f"{name}: {path} -> {real} {'writable' if writable else 'no permission (dialout?)'}")


def check_md400(r: Report, md_port: str, polls: int) -> dict:
    print(f"\n[2] MD400 — read-only, {polls} polls per controller (motors stay off)")
    from mdrobot import SingleMotorDriver
    from mdrobot.exceptions import MdrobotError

    out = {}
    for sid in (1, 2):
        rec = {"n": 0, "volt": [], "rpm": [], "alarm": 0, "pos": None, "ver": None, "err": "",
               "status": []}
        try:
            with SingleMotorDriver.open(md_port, slave_id=sid, timeout=0.3) as d:
                rec["ver"] = d.get_version()
                for _ in range(polls):
                    try:
                        m = d.read_monitor()
                        rec["n"] += 1
                        rec["rpm"].append(m.speed_rpm)
                        rec["pos"] = m.position
                        if m.status.raw:
                            rec["alarm"] += 1
                            rec["status"] = m.status.active
                        rec["volt"].append(d.get_voltage())
                    except MdrobotError as e:
                        rec["err"] = rec["err"] or f"{type(e).__name__}: {e}"
        except Exception as e:  # noqa: BLE001
            rec["err"] = f"{type(e).__name__}: {e}"
        out[sid] = rec
        side = "RIGHT" if sid == 1 else "LEFT"
        if rec["n"] == 0:
            r.ok(False, f"id={sid} ({side}): no answer — {rec['err']} (power / A-B wiring / GND)")
            continue
        v = statistics.fmean(rec["volt"]) if rec["volt"] else float("nan")
        rec["v"] = v
        r.ok(rec["n"] == polls, f"id={sid} ({side}): answered {rec['n']}/{polls}, fw v{rec['ver']}, "
                                f"{v:.3f} V, pos={rec['pos']}")
        r.ok(rec["alarm"] == 0, f"id={sid}: status alarms {rec['alarm']}"
                                + (f" — {rec['status']}" if rec["status"] else ""))
        r.ok(all(x == 0 for x in rec["rpm"]),
             f"id={sid}: at rest (max |rpm| {max(abs(x) for x in rec['rpm'])})")
        r.ok(20.0 < v < 30.0, f"id={sid}: bus voltage in range ({v:.3f} V)")
    if all("v" in out[s] for s in (1, 2)):
        gap = out[2]["v"] - out[1]["v"]
        r.ok(abs(gap - VOLT_GAP) < 0.25,
             f"controller voltmeter gap id2-id1 {gap:+.3f} V (reference {VOLT_GAP:+.3f} V, measurement offset)",
             blocking=False)
    return out


def check_pico(r: Report, pico_port: str, sec: float, rate: int, do_zero: bool, cal: dict) -> dict:
    print(f"\n[3] Pico — config, then {sec:.0f} s of rest data")
    import serial

    sp = serial.Serial(pico_port, 115200, timeout=0.2)
    fw, zero_line, rows = "?", "", []
    try:
        sp.write(b"X\n"); sp.flush(); time.sleep(0.3)
        sp.reset_input_buffer()
        sp.write(b"C\n"); sp.flush(); time.sleep(0.5)
        cfg = sp.read(4096).decode("utf-8", "replace")
        for ln in cfg.splitlines():
            ln = ln.strip()
            if ln.startswith("#CFG"):
                print(f"      {ln}")
                if "fw=" in ln:
                    fw = ln.split("fw=")[1].split()[0]
        r.ok(fw == EXPECTED_FW, f"firmware {fw} (expected {EXPECTED_FW})", blocking=False)

        sp.reset_input_buffer()
        sp.write(f"P{rate}\n".encode()); sp.flush(); time.sleep(0.3); sp.read(256)
        if do_zero:
            sp.write(b"Z\n"); sp.flush(); time.sleep(1.5)
            zero_line = sp.read(512).decode("utf-8", "replace").strip()
            print(f"      {zero_line}")
        sp.reset_input_buffer()

        sp.write(b"S\n"); sp.flush()
        buf, t_end = b"", time.monotonic() + sec
        while time.monotonic() < t_end:
            buf += sp.read(512)
            while b"\n" in buf:
                line, _, buf = buf.partition(b"\n")
                f = line.decode("utf-8", "replace").strip().split(",")
                if len(f) == 14 and f[0] == "D":
                    try:
                        rows.append((int(f[1]), int(f[2]) / 1e6, float(f[4]), float(f[7]),
                                     float(f[10]), int(f[13])))
                    except ValueError:
                        pass
    finally:
        try:
            sp.write(b"X\n"); sp.flush(); time.sleep(0.2); sp.close()
        except Exception:  # noqa: BLE001
            pass

    res = {"fw": fw, "zero": zero_line, "samples": len(rows)}
    if len(rows) < 10:
        r.ok(False, f"{len(rows)} samples — no stream")
        return res
    seqs = [x[0] for x in rows]
    dts = [b[1] - a[1] for a, b in zip(rows, rows[1:])]
    dt_ms = statistics.fmean(dts) * 1e3
    gaps = seqs[-1] - seqs[0] + 1 - len(seqs)
    over = sum(1 for x in rows if x[5] & 0x80)
    res.update({"dt_ms": dt_ms, "seq_gaps": gaps, "overrun_pct": 100.0 * over / len(rows)})
    r.ok(abs(dt_ms - 1000.0 / rate) < 1.0, f"sample interval {dt_ms:.3f} ms (rate {rate} Hz)")
    r.ok(gaps == 0, f"seq gaps {gaps} / {len(rows)} samples")
    r.ok(over / len(rows) < 0.03, f"overrun {over}/{len(rows)} ({res['overrun_pct']:.2f} %)", blocking=False)
    r.ok(not any(x[5] & 0x3F for x in rows), "all channels inside the linear range (raw 410..3686)")
    r.ok(all(x[5] & 0x40 for x in rows), "zero_valid set", blocking=False)

    print("\n[4] rest raw vs 2026-08-28 reference")
    cur, vol = cal["current"], cal["voltage"]
    gain = cur["a_per_lsb"] * cur["scale_gp27"]
    stats = {}
    for ch, idx in (("gp26", 2), ("gp27", 3), ("gp28", 4)):
        v = [x[idx] for x in rows]
        mean, sd = statistics.fmean(v), statistics.pstdev(v)
        stats[ch] = {"mean": mean, "sd": sd, "min": min(v), "max": max(v)}
        if ch == "gp26":
            vbus = (mean - vol["gp26_b_lsb"]) * vol["v_per_lsb"]
            stats[ch]["v_bus"] = vbus
            print(f"      gp26  raw {mean:8.2f}  sd {sd:5.2f}  -> bus {vbus:6.3f} V")
            r.ok(20.0 < vbus < 30.0, f"GP26 bus voltage {vbus:.3f} V in range")
            continue
        zero = cur[f"zero_{ch}"]
        lo, hi = REST_REF[ch]
        near = 0.0 if lo <= mean <= hi else min(abs(mean - lo), abs(mean - hi))
        side = "LEFT id2" if ch == "gp27" else "RIGHT id1"
        print(f"      {ch} ({side})  raw {mean:8.2f}  sd {sd:5.2f}  range {min(v):.0f}..{max(v):.0f}"
              f"  -> {(mean - zero) * gain:+.4f} A vs true zero (noise {sd * gain * 1e3:.1f} mA)")
        r.ok(near < REST_TOL, f"{ch} rest raw {mean:.2f} within {lo:.1f}..{hi:.1f} (+{near:.1f} LSB)",
             blocking=False)
        r.ok(sd * gain < 0.030, f"{ch} noise {sd * gain * 1e3:.1f} mA (target < 30 mA)", blocking=False)
    res["stats"] = stats
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--md-port", default=MD_PORT)
    ap.add_argument("--pico-port", default=PICO_PORT)
    ap.add_argument("--sec", type=float, default=15.0, help="Pico rest capture length")
    ap.add_argument("--polls", type=int, default=10, help="MD400 polls per controller")
    ap.add_argument("--rate", type=int, default=50)
    ap.add_argument("--no-zero", action="store_true", help="do not send Z (keep firmware zero)")
    ap.add_argument("--skip-md400", action="store_true")
    ap.add_argument("--skip-pico", action="store_true")
    ap.add_argument("--out-dir", default=str(records_dir() / "preflight"))
    a = ap.parse_args(argv)

    print("OROHA preflight — motors do not turn")
    r = Report()
    result = {"timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
              "timezone": time.strftime("%Z"), "args": vars(a)}
    check_ports(r, a.md_port, a.pico_port)
    if not a.skip_md400:
        try:
            result["md400"] = check_md400(r, a.md_port, a.polls)
        except Exception as e:  # noqa: BLE001
            r.ok(False, f"MD400 check failed: {type(e).__name__}: {e}")
    if not a.skip_pico:
        try:
            cal = _calibration()
            result["calib_id"] = cal.get("calib_id")
            result["pico"] = check_pico(r, a.pico_port, a.sec, a.rate, not a.no_zero, cal)
        except Exception as e:  # noqa: BLE001
            r.ok(False, f"Pico check failed: {type(e).__name__}: {e}")

    result["checks"] = r.checks
    result["ok"] = not r.failures
    print("\n" + "=" * 72)
    if r.failures:
        print(f"FAIL {len(r.failures)} / {len(r.checks)} — do not start a run:")
        for m in r.failures:
            print(f"  - {m}")
    if r.warnings:
        print(f"WARN {len(r.warnings)} — allowed, record in the test notes:")
        for m in r.warnings:
            print(f"  - {m}")
    if not r.failures:
        print(f"GO ({len(r.checks) - len(r.warnings)}/{len(r.checks)} passed)")

    out_dir = Path(a.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / (time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + ".json")
    with open(out, "w") as f:
        json.dump(result, f, indent=1, ensure_ascii=False, default=str)
    print(f"saved {out}")
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
