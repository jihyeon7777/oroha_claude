"""Pure measurement math: odometry comparison, hand-push constants, E-stop reading."""

import math

import pytest

from oroha_tools.direction_check import analyse
from oroha_tools.export_csv import odom_summary
from oroha_tools.wheel_push import compute


def test_odom_summary_uses_start_frame_and_unwrapped_yaw():
    # robot starts at (1, 2) heading +90 deg in odom, drives a full ccw circle of radius 0.5
    x0, y0, h0 = 1.0, 2.0, math.pi / 2
    samples = []
    for k in range(361):
        th = math.radians(k)
        # circle centre is to the left of the start heading
        cx, cy = x0 - 0.5, y0
        samples.append((cx + 0.5 * math.cos(th), cy + 0.5 * math.sin(th),
                        math.atan2(math.sin(h0 + th), math.cos(h0 + th))))
    s = odom_summary(samples, ideal_end={"x": 0.0, "y": 0.0, "yaw": 2 * math.pi})
    assert s["end_pose_rel"]["x"] == pytest.approx(0.0, abs=2e-4)
    assert s["end_pose_rel"]["y"] == pytest.approx(0.0, abs=2e-4)
    assert s["end_pose_rel"]["yaw_change_rad"] == pytest.approx(2 * math.pi, abs=2e-4)
    assert s["odom_minus_ideal"]["yaw_rad"] == pytest.approx(0.0, abs=2e-4)
    assert s["path_length_m"] == pytest.approx(math.pi, rel=1e-3)


def test_odom_summary_rotates_into_start_heading():
    # 1 m straight while heading +90 deg in odom = 1 m forward in the start frame
    s = odom_summary([(0.0, 0.0, math.pi / 2), (0.0, 1.0, math.pi / 2)],
                     gt={"x_m": 0.98, "y_m": 0.01})
    assert s["end_pose_rel"]["x"] == pytest.approx(1.0) and s["end_pose_rel"]["y"] == pytest.approx(0.0)
    assert s["odom_minus_gt"]["x"] == pytest.approx(0.02)


def test_wheel_push_reproduces_20260909():
    # counts from reports/hardware_test_20260909.md: 3 wheel revolutions -> 1045.8 counts/rev (+0.71 %)
    r = compute({1: -124, 2: -1622}, {1: 2996, 2: -4777}, revs=3)
    assert r["lin_counts"] == 3137.5 and r["mirror_sign_ok"]
    assert r["counts_per_wheel_rev"] == pytest.approx(1045.83, abs=0.01)
    assert r["diff_vs_ref_pct"] == pytest.approx(0.71, abs=0.01)


def test_estop_analysis_detects_stop_and_resume():
    rpms = [0, 0, 40, 98, 100, 100, 60, 2, 0, 0, 0, 50, 99, 100]
    a = analyse([{"t": i * 0.1, "rpm": r} for i, r in enumerate(rpms)], 100)
    assert a["first_motion_t"] == pytest.approx(0.2)
    assert len(a["stop_segments"]) == 1 and a["resumed_after_stop"]
    held = analyse([{"t": i * 0.1, "rpm": r} for i, r in enumerate([0, 50, 100, 100, 3, 0, 0, 0])], 100)
    assert held["stop_segments"][0]["resumed_t"] is None and not held["resumed_after_stop"]


def test_wheel_push_revs_and_dist_together():
    from oroha_tools.wheel_push import compute
    r = compute({1: 0, 2: 0}, {1: 3115, 2: -3115}, dist=2.372, revs=3)
    assert r["counts_per_wheel_rev"] == pytest.approx(1038.33, abs=0.01)
    assert r["m_per_count"] == pytest.approx(2.372 / 3115)
    assert r["rolling_circumference_m"] == pytest.approx(2.372 / 3)
    from oroha_tools.wheel_push import REF_M_PER_COUNT
    assert r["diff_vs_ref_pct"] == pytest.approx((2.372 / 3115 / REF_M_PER_COUNT - 1) * 100)
