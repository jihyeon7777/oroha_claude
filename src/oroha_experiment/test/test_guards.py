"""ROS-free tests for the per-wheel motion guard (plan review M2)."""

from oroha_experiment.guards import WheelGuard, describe


def _run(g, v, w, ml, mr, secs=2.0, dt=0.05):
    t, out = 0.0, None
    while t <= secs and out is None:
        out = g.update(t, v, w, ml, mr)
        t += dt
    return out, t


def test_expected_matches_diff_drive():
    g = WheelGuard()
    l, r = g.expected(0.2, 0.0)
    assert abs(l - 55.02) < 0.05 and abs(r - l) < 1e-9            # T20261001-03: 54.9 rad/s measured
    l, r = g.expected(0.0, 2.0)
    assert abs(r - 124.07) < 0.05 and abs(l + r) < 1e-9            # measured +123.5 / -124.3


def test_healthy_motion_and_ramp_lag_pass():
    g = WheelGuard()
    assert _run(g, 0.2, 0.0, 54.9, 54.9)[0] is None
    # first 0.8 s still at rest (diff_cont ramp), then turning: no verdict
    t, v = 0.0, None
    while t < 3.0:
        m = 0.0 if t < 0.8 else 55.0
        v = v or g.update(t, 0.2, 0.0, m, m)
        t += 0.05
    assert v is None


def test_one_dead_side_is_caught_after_timeout():
    g = WheelGuard()
    v, t = _run(g, 0.2, 0.0, 0.3, 54.9)
    assert v is not None and v[0] == "NOT_TURNING" and v[1] == "left"
    assert 1.0 < t <= 1.15


def test_wrong_way_and_small_demands():
    g = WheelGuard()
    v, _ = _run(g, 0.0, 1.0, 31.0, 31.0)       # spot turn CCW: left should go backwards
    assert v and v[0] == "WRONG_WAY" and v[1] == "left"
    g.reset()
    assert _run(WheelGuard(), 0.02, 0.0, 0.0, 0.0)[0] is None       # 5.5 rad/s demand: not checked


def test_describe_hints():
    v = ("NOT_TURNING", "left", 55.0, 0.1, 1.05)
    assert "E-stop" in describe(v, currents=(0.08, 0.09), status=(0, 0))
    assert "blocked" in describe(v, currents=(0.9, 0.3))
    assert "comm lost" in describe(v, currents=(0.09, 0.35))
    assert "alarm" in describe(v, status=(0x01, 0))
