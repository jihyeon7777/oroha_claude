"""Geometry checks for the open-loop path profiles (no ROS)."""

import math

import pytest

from oroha_experiment import profiles as P

DT = 0.05


def _end(p):
    return p.ideal_path(DT)[-1]


def test_straight_length_and_heading():
    p = P.straight(2.0, 0.3)
    e = _end(p)
    assert abs(e.x - 2.0) < 2e-3
    assert abs(e.y) < 1e-6
    assert abs(e.yaw) < 1e-9
    assert abs(p.path_length(DT) - 2.0) < 2e-3


def test_straight_backward():
    e = _end(P.straight(-1.0, 0.2))
    assert abs(e.x + 1.0) < 2e-3


def test_short_straight_is_triangle_but_exact():
    # 0.05 m at 0.3 m/s with 0.3 m/s^2 cannot reach 0.3 m/s
    p = P.straight(0.05, 0.3)
    seg = p.segments[1]
    assert seg.v_peak < 0.3
    assert abs(_end(p).x - 0.05) < 1e-3


def test_circle_closes_and_turns_2pi():
    for direction, sign in (("ccw", 1.0), ("cw", -1.0)):
        p = P.circle(0.75, 0.3, direction=direction)
        e = _end(p)
        assert math.hypot(e.x, e.y) < 5e-3, (direction, e)
        assert abs(e.yaw - sign * 2.0 * math.pi) < 2e-3
        assert abs(p.path_length(DT) - 2.0 * math.pi * 0.75) < 5e-3


def test_square_spot_closes():
    p = P.square(1.0, 0.3, corner="spot", turn_w=0.5)
    e = _end(p)
    assert math.hypot(e.x, e.y) < 5e-3, e
    assert abs(e.yaw - 2.0 * math.pi) < 2e-3
    assert abs(p.path_length(DT) - 4.0) < 5e-3
    labels = [s.label for s in p.segments]
    assert labels[0] == "rest_pre" and labels[-1] == "rest_post"
    assert labels.count("side1") == 1 and "turn4" in labels


def test_square_arc_closes():
    p = P.square(1.5, 0.3, corner="arc", corner_radius=0.3)
    e = _end(p)
    assert math.hypot(e.x, e.y) < 5e-3, e
    assert abs(e.yaw - 2.0 * math.pi) < 2e-3


def test_s_curve_end_pose():
    r = 0.6
    p = P.s_curve(r, 0.25, arc_deg=180.0, first="left", join="continuous")
    e = _end(p)
    # continuous join: the inflection is quantised to the 20 Hz command hold
    assert abs(e.x) < 2e-2 and abs(e.y - 4.0 * r) < 2e-2, e
    assert abs(e.yaw) < 1.5e-2
    p2 = P.s_curve(r, 0.25, arc_deg=180.0, first="right", join="stop")
    e2 = _end(p2)
    assert abs(e2.x) < 5e-3 and abs(e2.y + 4.0 * r) < 5e-3, e2


def test_limits_rejected():
    with pytest.raises(ValueError):
        P.straight(2.0, 1.5)                       # > v_max
    with pytest.raises(ValueError):
        P.circle(0.2, 0.7)                         # wheel speed 0.7 + 3.5*0.2255 > 1.0
    with pytest.raises(ValueError):
        P.square(0.4, 0.3, corner="arc", corner_radius=0.3)


def test_ramps_continuous():
    p = P.square(1.0, 0.3, corner="spot", turn_w=0.6)
    s = p.sample(DT)
    for a, b in zip(s, s[1:]):
        assert abs(b.v - a.v) <= p.limits.accel * DT + 1e-9
        assert abs(b.w - a.w) <= p.limits.ang_accel * DT + 1e-9
    assert s[0].v == 0.0 and s[-1].v == 0.0


def test_rows_and_csv(tmp_path):
    p = P.build("straight", length=1.0, v=0.2)
    rows = p.rows(DT)
    assert list(rows[0].keys()) == ["t", "v", "w", "seg_idx", "seg_label", "x_ideal", "y_ideal", "yaw_ideal"]
    out = tmp_path / "p.csv"
    p.write_csv(str(out), DT)
    assert out.read_text().splitlines()[0] == "t,v,w,seg_idx,seg_label,x_ideal,y_ideal,yaw_ideal"


def test_cli_runs(capsys):
    assert P.main(["square", "--side", "1.0", "--v", "0.2"]) == 0
    assert "duration" in capsys.readouterr().out


def test_spot_turns_in_place():
    p = P.spot(360.0, turn_w=0.5)
    e = _end(p)
    assert math.hypot(e.x, e.y) < 1e-9
    assert abs(e.yaw - 2.0 * math.pi) < 2e-3
    assert p.path_length(DT) == 0.0
    with pytest.raises(ValueError):
        P.spot(360.0, turn_w=3.0)                  # > w_max


def test_footprint_and_arena():
    half = 0.36          # robot body half-diagonal (placeholder until measured)
    fp = P.footprint(P.straight(2.0, 0.2), half)
    assert fp.size == pytest.approx((2.0 + 2 * half, 2 * half), abs=3e-3)
    # 3 m room, 0.3 m margins -> 2.4 m usable: a 2 m straight needs 2.72 m along a wall
    assert fp.placement(3.0, 0.3) == "none"
    assert P.footprint(P.straight(1.5, 0.2), half).placement(3.0, 0.3) == "wall"
    assert P.footprint(P.straight(1.9, 0.2), half).placement(3.0, 0.3) == "diagonal"
    # S at R0.6 needs 4R + body = 3.12 m; R0.4 fits along the walls
    assert P.footprint(P.s_curve(0.6, 0.25), half).placement(3.0, 0.3) == "none"
    assert P.footprint(P.s_curve(0.4, 0.2), half).placement(3.0, 0.3) == "wall"
    assert P.footprint(P.circle(0.75, 0.3), half).placement(3.0, 0.3) == "wall"
    assert P.footprint(P.spot(360.0), half).placement(3.0, 0.3) == "wall"
    # start offset puts the whole footprint inside [margin, arena - margin]
    fp = P.footprint(P.circle(0.75, 0.3, direction="cw"), half)
    ox, oy = fp.start_offset(0.3)
    assert fp.xmin + ox == pytest.approx(0.3) and fp.ymin + oy == pytest.approx(0.3)
    assert fp.xmax + ox <= 2.7 + 1e-9 and fp.ymax + oy <= 2.7 + 1e-9


def test_body_rectangle_footprint_measured_robot():
    """0.80 x 0.53 m body (T20261006-05) in the 3 m room with 0.3 m margins (2.4 m usable)."""
    body = (0.80, 0.53)
    fp = P.footprint(P.straight(1.5, 0.2), body)
    assert fp.size == pytest.approx((1.5 + 0.80, 0.53), abs=3e-3)    # no half-diagonal waste
    assert fp.placement(3.0, 0.3) == "wall"
    assert P.footprint(P.straight(2.0, 0.2), body).placement(3.0, 0.3) == "diagonal"
    sp = P.footprint(P.spot(360.0), body)                           # a full turn sweeps the circle
    d = math.hypot(0.80, 0.53)
    assert sp.size == pytest.approx((d, d), abs=5e-3)
    assert P.footprint(P.square(1.0, 0.2), body).placement(3.0, 0.3) == "wall"
    # the circle model is still available and more conservative
    assert P.footprint(P.straight(1.5, 0.2), 0.48).placement(3.0, 0.3) == "none"
