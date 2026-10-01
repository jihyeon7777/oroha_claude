"""ROS-free tests for the run-level current reference (plan H3)."""

from oroha_power.protocol import Calibration
from oroha_tools.power_analysis import analyse, rest_windows

CAL = Calibration(calib_id="t", a_per_lsb=12.133e-3, scale_gp27=0.94289, scale_gp28=0.94289,
                  sign_gp27=1, sign_gp28=1, zero_gp27=2035.257, zero_gp28=2033.974,
                  v_per_lsb=8.913e-3, gp26_b_lsb=-18.7, quiet_a=0.080)
K = 12.133e-3 * 0.94289


def _profile(pre=2.5, run=5.0, post=2.0, dt=0.05):
    rows, t = [], 0.0
    for lab, dur in (("rest_pre", pre), ("side1", run), ("rest_post", post)):
        n = int(round(dur / dt))
        for _ in range(n):
            rows.append({"t": f"{t:.2f}", "seg_label": lab})
            t += dt
    return rows


def _rows(rc, motor_lsb, t_end=9.5, dt=0.02):
    """Powered rest at rail ratio rc; +motor_lsb on the LEFT channel during 2.5..7.5 s."""
    z27, z28, q = CAL.zero_gp27 / rc, CAL.zero_gp28 / rc, CAL.quiet_lsb / rc
    rows, t, ns = [], 0.0, 0
    while t < t_end:
        moving = 2.5 <= t < 7.5
        rows.append({"t_dev": t, "dev_ns": ns, "gp26_mean": 3123.0 / rc,
                     "gp27_mean": z27 + q + (motor_lsb if moving else 0.0), "gp28_mean": z28 + q,
                     "i_left": 0.0, "i_right": 0.0, "p_total": 0.0, "v_bus": 0.0})
        t += dt
        ns += int(dt * 1e9)
    return rows


def test_rest_windows_trimmed():
    w = rest_windows(_profile())
    assert [x[0] for x in w] == ["rest_pre", "rest_post"]
    assert abs(w[0][1] - 0.3) < 1e-6 and abs(w[0][2] - (2.45 - 0.1)) < 1e-6   # pre: +0.3 / -0.1 s
    assert abs(w[1][1] - (7.5 + 1.0)) < 1e-6                                   # post starts 1.0 s late
    assert rest_windows([{"t": "0", "seg_label": "rest_pre"}]) == []          # too short


def test_rail_shift_is_removed_and_meanings_hold():
    rc, motor = 1.0026, 20.0
    rows = _rows(rc, motor)
    s = analyse(rows, CAL, rest_windows(_profile()))
    ref = s["current_reference"]
    assert len(ref["windows"]) == 2
    assert all(abs(w["rail_corr"] - rc) < 1e-4 for w in ref["windows"])
    assert abs(ref["baseline_drift_lsb"]["gp27"]) < 1e-6
    rest = [r for r in rows if r["t_dev"] < 2.4]
    mov = [r for r in rows if 3.0 < r["t_dev"] < 7.0]
    assert all(abs(r["i_left_abs"] - 0.080) < 1e-3 and abs(r["di_left"]) < 1e-3 for r in rest)
    assert all(abs(r["di_left"] - motor * K * rc) < 1e-3 for r in mov)          # 20 LSB ~ 0.229 A
    assert all(abs(r["di_right"]) < 1e-3 and abs(r["i_right_abs"] - 0.080) < 1e-3 for r in mov)
    assert abs(rows[0]["v_bus_run"] - (3123.0 / rc + 18.7) * 8.913e-3 * rc) < 1e-3
    # energy: increase only on the left, for ~5 s at v * 0.229 A
    v = rows[0]["v_bus_run"]
    e = s["energy_wh"]
    assert abs(e["inc_left"] - v * motor * K * rc * 5.0 / 3600) < 2e-4
    assert abs(e["inc_right"]) < 1e-6
    assert abs(e["abs_total"] - (e["inc_total"] + v * 2 * 0.080 * 9.5 / 3600)) < 3e-4
    assert s["range"]["frac_above"] == {"left": 0.0, "right": 0.0}


def test_no_rest_window_falls_back_to_node_values():
    rows = _rows(1.0, 0.0)
    s = analyse(rows, CAL, [])
    assert s["current_reference"]["windows"] == []
    assert rows[0]["di_left"] == "" and "inc_total" not in s["energy_wh"]


def test_common_mode_dip_is_left_out_of_the_baseline():
    rows = _rows(1.0, 0.0)
    for r in rows:                      # a process start at 1.0..1.8 s: all channels -6 LSB
        if 1.0 <= r["t_dev"] < 1.8:
            for k in ("gp26_mean", "gp27_mean", "gp28_mean"):
                r[k] -= 6.0
    s = analyse(rows, CAL, rest_windows(_profile()))
    w = s["current_reference"]["windows"][0]
    assert w["common_mode_excluded"] == 40
    assert abs(w["raw_gp27"] - (CAL.zero_gp27 + CAL.quiet_lsb)) < 1e-3
