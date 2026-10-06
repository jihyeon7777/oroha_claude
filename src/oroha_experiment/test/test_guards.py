"""ROS-free tests for the per-wheel motion guard (plan review M2)."""

from oroha_experiment.guards import WheelGuard, describe, gate_open


def _run(g, v, w, ml, mr, secs=2.0, dt=0.05):
    t, out = 0.0, None
    while t <= secs and out is None:
        out = g.update(t, v, w, ml, mr)
        t += dt
    return out, t


def test_expected_matches_diff_drive():
    g = WheelGuard(wheel_radius=0.003635, wheel_separation=0.451)   # values in use on 2026-10-01 (T20261001-03)
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


def test_not_tracking_catches_a_dragged_side():
    """T20261006-07: left lost drive in a forward-left curve and was dragged at 63 rad/s."""
    g = WheelGuard()
    l, r = g.expected(0.15, 0.5)                    # ~10.4 / ~73.4 rad/s
    v, t = _run(g, 0.15, 0.5, 63.0, r, secs=3.0)
    assert v and v[0] == "NOT_TRACKING" and v[1] == "left" and 1.5 < t <= 1.65
    g = WheelGuard()
    v, _ = _run(g, -0.15, 0.0, -38.0, -3.8, secs=3.0)   # right dragged backwards at -3.8 (>= still)
    assert v and v[0] == "NOT_TRACKING" and v[1] == "right"
    g = WheelGuard()                                  # 20 % slow under load: tolerated
    assert _run(g, 0.2, 0.0, 0.8 * 55.9, 0.8 * 55.9, secs=3.0)[0] is None


def test_gate_open_hint():
    assert gate_open(0x34) is False and gate_open(0x24) is True and gate_open(0) is False
    v = ("NOT_TRACKING", "left", 10.4, 63.0, 1.6)
    msg = describe(v, currents=(0.11, 1.1), status=(0, 0), di=(0x24, 0x34))
    assert "CTRL gate open on LEFT" in msg and msg.index("CTRL gate") < msg.index("MD400 not driving")
