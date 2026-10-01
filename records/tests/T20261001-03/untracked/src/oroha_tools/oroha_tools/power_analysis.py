"""Run-level current reference (plan H3, 계측_GT_연동 §1): true 0 A vs powered-rest baseline.

Why per run: the ACS37030 is non-ratiometric, so its raw 0 A point moves with the Pico 3.3 V
rail (preflight powered-rest raw moved -10..+3 LSB between days on this Pi = up to ~0.15 A).
The run's own rest windows (rest_pre / rest_post, motors enabled at 0 rpm) are a powered-rest
zero measurement taken minutes - not days - from the data:

  baseline raw b(t)   per channel, mean raw over each rest window, linear between windows
  rail_corr(t)        rail_corr_from_rest(b27, b28)  (firmware '#ZERO' model)
  di  = (raw - b) * k * rail_corr          increase over powered rest  [A]  (motor-attributable)
  i_abs = di + quiet_a                     vs TRUE 0 A                 [A]  (assumes the 80 mA
                                                                              calibrated quiescent)
  v_bus = (raw26 - b_lsb) * v_per_lsb * rail_corr

Common mode (T20261001-03): whenever a process starts on the Pi (ros2 CLI, launch, the bag
recorder) all three channels dip together by 2..7 LSB for 1..2 s with the motors at rest (Pi
load moving the measurement ground; current/voltage dip ratio ~0.8). At rest the bus voltage is
constant, so rest samples whose GP26 raw is more than CM_TOL_LSB from the window median are
left out of the baseline (counted in the summary). During motion it cannot be separated from a
real voltage sag: a run that starts a process mid-run carries up to ~0.08 A of transient error.

Uncertainty (sensing-20260828): gain +-1.5 %, run-to-run +-2..3 %, quiescent +-3 mA. The
pre->post baseline drift is reported so a run whose rest windows disagree can be spotted.
Energies integrate on device time; intervals outside (0, 60 ms] are excluded and counted.
"""

from __future__ import annotations

import os
import statistics
from pathlib import Path

import yaml
from oroha_power.protocol import Calibration, rail_corr_from_rest

REST_LABELS = ("rest_pre", "rest_post")
# seconds cut from the (start, end) of each window: rest_post starts while diff_cont still ramps down
REST_TRIM = {"rest_pre": (0.3, 0.1), "rest_post": (1.0, 0.1)}
MIN_REST_S = 0.5
MIN_REST_SAMPLES = 10
CM_TOL_LSB = 2.5          # GP26 rest noise is ~1.3 LSB (window mean); dips are 2..7 LSB
MAX_DT_S = 0.060
NEG_TOL_A = 0.05          # ~4 sigma of the 13.7 mA rest noise


def load_calibration(calib_id: str) -> Calibration:
    """config/calibration/<calib_id>.yaml from the installed oroha_power, else the source tree."""
    cands = []
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(Path(get_package_share_directory("oroha_power")) / "config" / "calibration")
    except Exception:  # noqa: BLE001  (no ROS environment: unit tests)
        pass
    cands.append(Path(__file__).resolve().parents[2] / "oroha_power" / "config" / "calibration")
    for d in cands:
        p = d / f"{calib_id}.yaml"
        if p.exists():
            with open(p) as f:
                return Calibration.from_dict(yaml.safe_load(f))
    raise FileNotFoundError(f"calibration {calib_id} not found in {[str(c) for c in cands]}")


def rest_windows(profile_rows, labels=REST_LABELS, trim=REST_TRIM) -> list:
    """[(label, t0, t1)] from profile.csv rows (t, seg_label), contiguous segments, trimmed."""
    out, cur = [], None
    for r in profile_rows:
        lab, t = r["seg_label"], float(r["t"])
        if cur and lab == cur[0]:
            cur[2] = t
            continue
        if cur:
            out.append(tuple(cur))
        cur = [lab, t, t] if lab in labels else None
    if cur:
        out.append(tuple(cur))
    res = []
    for lab, a, b in out:
        da, db = trim.get(lab, (0.0, 0.0))
        if (b - db) - (a + da) >= MIN_REST_S:
            res.append((lab, a + da, b - db))
    return res


def _interp(refs, t):
    if len(refs) == 1 or t <= refs[0][0]:
        return refs[0][1:]
    if t >= refs[-1][0]:
        return refs[-1][1:]
    for (ta, *a), (tb, *b) in zip(refs, refs[1:]):
        if ta <= t <= tb:
            u = (t - ta) / (tb - ta)
            return tuple(x + u * (y - x) for x, y in zip(a, b))
    return refs[-1][1:]


def analyse(rows: list, cal: Calibration, windows: list) -> dict:
    """Add rest-referenced columns to `rows` (in place) and return the summary block.

    rows: dicts in time order with t_dev (s, device time relative to START), dev_ns,
          gp26_mean, gp27_mean, gp28_mean, i_left, i_right, p_total (node values).
    """
    refs, used = [], []
    for lab, a, b in windows:
        allsel = [r for r in rows if a <= r["t_dev"] <= b]
        if len(allsel) < MIN_REST_SAMPLES:
            continue
        med26 = statistics.median(r["gp26_mean"] for r in allsel)
        sel = [r for r in allsel if abs(r["gp26_mean"] - med26) <= CM_TOL_LSB]
        if len(sel) < MIN_REST_SAMPLES:
            sel = allsel
        b27 = statistics.fmean(r["gp27_mean"] for r in sel)
        b28 = statistics.fmean(r["gp28_mean"] for r in sel)
        rc = rail_corr_from_rest(cal, b27, b28)
        tc = statistics.fmean(r["t_dev"] for r in sel)
        refs.append((tc, b27, b28, rc))
        used.append({"label": lab, "t0": round(a, 3), "t1": round(b, 3), "samples": len(sel),
                     "common_mode_excluded": len(allsel) - len(sel),
                     "raw_gp27": round(b27, 3), "raw_gp28": round(b28, 3), "rail_corr": round(rc, 6),
                     "i_node_left_a": round(statistics.fmean(r["i_left"] for r in sel), 4),
                     "i_node_right_a": round(statistics.fmean(r["i_right"] for r in sel), 4)})
    refs.sort()
    k27, k28 = cal.k("gp27"), cal.k("gp28")
    lo, hi = cal.valid_range_a

    keys = ("rail_corr_run", "base_gp27", "base_gp28", "v_bus_run", "i_left_abs", "i_right_abs",
            "di_left", "di_right", "p_total_abs", "p_total_inc")
    e = {"abs_left": 0.0, "abs_right": 0.0, "abs_total": 0.0, "inc_left": 0.0, "inc_right": 0.0,
         "inc_total": 0.0, "node_total": 0.0}
    n_excl, t_excl, prev_ns = 0, 0.0, None
    above = {"left": 0, "right": 0}
    below = {"left": 0, "right": 0}
    sums = {"i_left_abs": 0.0, "i_right_abs": 0.0, "di_left": 0.0, "di_right": 0.0}
    peaks = {"i_left_abs": 0.0, "i_right_abs": 0.0}
    for r in rows:
        if refs:
            b27, b28, rc = _interp(refs, r["t_dev"])
            v = (r["gp26_mean"] - cal.gp26_b_lsb) * cal.v_per_lsb * cal.scale_v * rc
            dl = (r["gp27_mean"] - b27) * k27 * rc
            dr = (r["gp28_mean"] - b28) * k28 * rc
            il, ir = dl + cal.quiet_a, dr + cal.quiet_a
            r.update({"rail_corr_run": round(rc, 6), "base_gp27": round(b27, 3), "base_gp28": round(b28, 3),
                      "v_bus_run": round(v, 4), "i_left_abs": round(il, 5), "i_right_abs": round(ir, 5),
                      "di_left": round(dl, 5), "di_right": round(dr, 5),
                      "p_total_abs": round(v * (il + ir), 4), "p_total_inc": round(v * (dl + dr), 4)})
        else:
            v, il, ir = r.get("v_bus", 0.0), r["i_left"], r["i_right"]
            dl = dr = None
            r.update({k: "" for k in keys})
        for side, i in (("left", il), ("right", ir)):
            above[side] += i > hi
            below[side] += i < lo - NEG_TOL_A
        sums["i_left_abs"] += il
        sums["i_right_abs"] += ir
        peaks["i_left_abs"] = max(peaks["i_left_abs"], il)
        peaks["i_right_abs"] = max(peaks["i_right_abs"], ir)
        if dl is not None:
            sums["di_left"] += dl
            sums["di_right"] += dr
        if prev_ns is not None:
            dt = (r["dev_ns"] - prev_ns) * 1e-9
            if 0.0 < dt <= MAX_DT_S:
                h = dt / 3600.0
                e["node_total"] += r["p_total"] * h
                e["abs_left"] += v * il * h
                e["abs_right"] += v * ir * h
                if dl is not None:
                    e["inc_left"] += v * dl * h
                    e["inc_right"] += v * dr * h
            else:
                n_excl += 1
                t_excl += max(dt, 0.0)
        prev_ns = r["dev_ns"]
    e["abs_total"] = e["abs_left"] + e["abs_right"]
    e["inc_total"] = e["inc_left"] + e["inc_right"]
    n = max(1, len(rows))
    out = {
        "current_reference": {
            "method": ("rest windows of this run (powered-rest baseline + rail_corr), linear between windows"
                       if refs else "none: no rest window -> node values (rail_corr as published)"),
            "windows": used,
            "baseline_drift_lsb": None if len(refs) < 2 else {
                "gp27": round(refs[-1][1] - refs[0][1], 3), "gp28": round(refs[-1][2] - refs[0][2], 3)},
            "quiet_a_assumed": cal.quiet_a, "common_mode_tol_lsb": CM_TOL_LSB,
            "i_abs": "di + quiet_a (vs true 0 A)", "di": "increase over powered rest (motor-attributable)",
        },
        "energy_wh": {k: round(x, 5) for k, x in e.items()
                      if refs or not k.startswith("inc")},
        "energy_method": "sum(p*dt) on device_stamp, current sample's power; intervals outside (0, 60 ms] excluded",
        "excluded_intervals": n_excl, "excluded_time_s": round(t_excl, 3),
        "mean_a": {k: round(x / n, 4) for k, x in sums.items() if refs or not k.startswith("di")},
        "max_a": {k: round(x, 4) for k, x in peaks.items()},
        "range": {"valid_a": list(cal.valid_range_a),
                  "frac_above": {s: round(c / n, 4) for s, c in above.items()},
                  "frac_below": {s: round(c / n, 4) for s, c in below.items()},
                  "note": "outside the calibrated range = extrapolation (negative = regeneration)"},
    }
    return out
