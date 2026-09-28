"""Time-based open-loop velocity profiles for the OROHA paper experiments.

Pure Python, no ROS. A Profile is an ordered list of Segments; each segment
commands body velocity (v [m/s], w [rad/s]) as a function of time with
symmetric trapezoidal ramps. Because arcs scale v and w together, the
commanded curvature is constant during ramps and the ideal path is exactly the
geometric shape (straight line, circle, square, S) under a no-slip assumption.

Paths (see build()):
  straight(length, v)                       one line
  circle(radius, v, direction)              one full circle, ccw|cw
  square(side, v, corner, turn_w, ...)      4 sides; corners are spot turns or arcs
  s_curve(radius, v, arc_deg, first, join)  left arc then right arc (or reverse)

Every path is wrapped in pre/post rest segments (local-zero anchors for the
current measurement) and a dwell between motion segments.

CLI:  oroha_profile square --side 1.5 --v 0.3 --csv /tmp/square.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import asdict, dataclass, field
from typing import Dict, Iterable, List, Sequence, Tuple

DT_DEFAULT = 0.05  # 20 Hz command rate


@dataclass(frozen=True)
class Limits:
    """Command limits; validate() rejects a profile that exceeds any of them."""
    v_max: float = 0.8            # m/s, body
    w_max: float = 2.0            # rad/s, body
    accel: float = 0.3            # m/s^2, linear ramp
    ang_accel: float = 1.0        # rad/s^2, angular ramp (spot turns)
    wheel_v_max: float = 1.0      # m/s, per side (3000 rpm = 1.14 m/s)
    half_track: float = 0.2255    # m, wheel_separation / 2


# ---------------------------------------------------------------- trapezoid --

def _shape(total: float, ramp: float, t: float) -> float:
    """Shape factor in [0, 1]: linear ramps of length `ramp` at both ends."""
    if t <= 0.0 or t >= total:
        return 0.0
    if ramp <= 0.0:
        return 1.0
    return min(min(t, total - t) / ramp, 1.0)


def _shape_area(total: float, ramp: float) -> float:
    """Integral of _shape over [0, total] (factor-seconds)."""
    if ramp <= 0.0:
        return total
    if total >= 2.0 * ramp:
        return total - ramp
    return total * total / (4.0 * ramp)          # triangle


def _shape_integral(total: float, ramp: float, t: float) -> float:
    """Integral of _shape over [0, t]."""
    if t <= 0.0:
        return 0.0
    if t >= total:
        return _shape_area(total, ramp)
    if ramp <= 0.0:
        return t
    if total < 2.0 * ramp:
        ramp = total / 2.0                       # triangle: ramps meet at the middle
    if t <= ramp:
        return t * t / (2.0 * ramp)
    if t <= total - ramp:
        return ramp / 2.0 + (t - ramp)
    return _shape_area(total, ramp) - (total - t) ** 2 / (2.0 * ramp)


def _plan(amount: float, peak: float, accel: float) -> Tuple[float, float, float]:
    """(duration, reached_peak, ramp) to cover `amount` at `peak` rate with `accel` ramps.

    Trapezoid when the distance allows reaching `peak`, otherwise a triangle
    whose peak is lower. Exact: area == amount in both cases.
    """
    if amount <= 0.0 or peak <= 0.0 or accel <= 0.0:
        raise ValueError(f"amount, peak, accel must be > 0 (got {amount}, {peak}, {accel})")
    ramp = peak / accel
    if amount / peak >= ramp:
        return amount / peak + ramp, peak, ramp
    total = 2.0 * math.sqrt(amount / accel)
    return total, accel * total / 2.0, total / 2.0


# ------------------------------------------------------------------ segment --

@dataclass(frozen=True)
class Segment:
    label: str
    kind: str                 # rest | straight | arc | spot | s_curve
    duration: float           # s
    v_peak: float = 0.0       # m/s (signed: negative = backward)
    w_peak: float = 0.0       # rad/s (signed: + = ccw/left). arc: v_peak / R
    ramp: float = 0.0         # s, ramp length at each end

    def cmd(self, t: float) -> Tuple[float, float]:
        """Body velocity command at segment-local time t."""
        if self.kind == "rest":
            return 0.0, 0.0
        f = _shape(self.duration, self.ramp, t)
        if self.kind == "s_curve":
            # curvature flips at half the arc length. With a 20 Hz command hold the
            # flip lands up to dt/2 late/early -> heading error <= |w|*dt/2 (~0.5 deg)
            # in the ideal path; this is the commanded path, not a bug.
            half = _shape_area(self.duration, self.ramp) / 2.0
            sign = 1.0 if _shape_integral(self.duration, self.ramp, t) < half else -1.0
            return self.v_peak * f, sign * self.w_peak * f
        return self.v_peak * f, self.w_peak * f


def rest(duration: float, label: str = "rest") -> Segment:
    return Segment(label, "rest", duration)


def straight_segment(length: float, v: float, accel: float, label: str = "straight") -> Segment:
    total, peak, ramp = _plan(abs(length), abs(v), accel)
    return Segment(label, "straight", total, math.copysign(peak, length), 0.0, ramp)


def arc_segment(radius: float, angle: float, v: float, accel: float, label: str = "arc") -> Segment:
    """Arc of `radius` through `angle` rad (+ = left/ccw) at |v|; curvature constant."""
    if radius <= 0.0:
        raise ValueError("radius must be > 0")
    total, peak, ramp = _plan(radius * abs(angle), abs(v), accel)
    return Segment(label, "arc", total, peak, math.copysign(peak / radius, angle), ramp)


def spot_segment(angle: float, w: float, ang_accel: float, label: str = "spot") -> Segment:
    """Turn in place through `angle` rad (+ = ccw) at |w| rad/s."""
    total, peak, ramp = _plan(abs(angle), abs(w), ang_accel)
    return Segment(label, "spot", total, 0.0, math.copysign(peak, angle), ramp)


def s_curve_segment(radius: float, arc_angle: float, v: float, accel: float,
                    first_left: bool, label: str = "s") -> Segment:
    """Two joined arcs of equal length and opposite curvature, no stop between."""
    if radius <= 0.0:
        raise ValueError("radius must be > 0")
    total, peak, ramp = _plan(2.0 * radius * abs(arc_angle), abs(v), accel)
    w = peak / radius * (1.0 if first_left else -1.0)
    return Segment(label, "s_curve", total, peak, w, ramp)


# ------------------------------------------------------------------ profile --

@dataclass
class Sample:
    t: float
    v: float
    w: float
    seg_idx: int
    seg_label: str


@dataclass
class Pose:
    t: float
    x: float
    y: float
    yaw: float


@dataclass
class Profile:
    name: str
    params: Dict[str, object]
    segments: List[Segment]
    limits: Limits = field(default_factory=Limits)

    @property
    def duration(self) -> float:
        return sum(s.duration for s in self.segments)

    def cmd_at(self, t: float) -> Tuple[float, float, int, str]:
        """(v, w, segment index, label) at profile time t; zeros outside [0, duration)."""
        if t < 0.0:
            return 0.0, 0.0, -1, ""
        t0 = 0.0
        for i, s in enumerate(self.segments):
            if t < t0 + s.duration:
                v, w = s.cmd(t - t0)
                return v, w, i, s.label
            t0 += s.duration
        return 0.0, 0.0, -1, ""

    def sample(self, dt: float = DT_DEFAULT) -> List[Sample]:
        n = int(math.ceil(self.duration / dt)) + 1
        out = []
        for k in range(n):
            t = k * dt
            v, w, i, label = self.cmd_at(t)
            out.append(Sample(t, v, w, i, label))
        return out

    def ideal_path(self, dt: float = DT_DEFAULT) -> List[Pose]:
        """Pose the commands describe under no slip (exact arc step, midpoint command)."""
        x = y = yaw = 0.0
        poses = [Pose(0.0, 0.0, 0.0, 0.0)]
        n = int(math.ceil(self.duration / dt))
        for k in range(n):
            v, w, _, _ = self.cmd_at((k + 0.5) * dt)
            if abs(w) < 1e-9:
                x += v * dt * math.cos(yaw)
                y += v * dt * math.sin(yaw)
            else:
                yaw1 = yaw + w * dt
                x += v / w * (math.sin(yaw1) - math.sin(yaw))
                y -= v / w * (math.cos(yaw1) - math.cos(yaw))
                yaw = yaw1
            poses.append(Pose((k + 1) * dt, x, y, yaw))
        return poses

    def path_length(self, dt: float = DT_DEFAULT) -> float:
        return sum(abs(self.cmd_at((k + 0.5) * dt)[0]) * dt
                   for k in range(int(math.ceil(self.duration / dt))))

    def validate(self) -> None:
        """Raise ValueError if any command exceeds the limits or the profile is malformed."""
        lim = self.limits
        if not self.segments:
            raise ValueError("empty profile")
        for s in self.segments:
            if s.duration <= 0.0:
                raise ValueError(f"segment {s.label}: duration must be > 0")
            if abs(s.v_peak) > lim.v_max + 1e-9:
                raise ValueError(f"segment {s.label}: |v| {abs(s.v_peak):.3f} > v_max {lim.v_max}")
            if abs(s.w_peak) > lim.w_max + 1e-9:
                raise ValueError(f"segment {s.label}: |w| {abs(s.w_peak):.3f} > w_max {lim.w_max}")
            wheel = abs(s.v_peak) + abs(s.w_peak) * lim.half_track
            if wheel > lim.wheel_v_max + 1e-9:
                raise ValueError(f"segment {s.label}: wheel speed {wheel:.3f} > {lim.wheel_v_max} m/s")
        first, last = self.segments[0], self.segments[-1]
        if first.kind != "rest" or last.kind != "rest":
            raise ValueError("profile must start and end with a rest segment")

    def rows(self, dt: float = DT_DEFAULT) -> List[Dict[str, object]]:
        """CSV rows: t, v, w, seg_idx, seg_label, x_ideal, y_ideal, yaw_ideal."""
        samples = self.sample(dt)
        poses = self.ideal_path(dt)
        out = []
        for s, p in zip(samples, poses):
            out.append({"t": round(s.t, 6), "v": round(s.v, 6), "w": round(s.w, 6),
                        "seg_idx": s.seg_idx, "seg_label": s.seg_label,
                        "x_ideal": round(p.x, 6), "y_ideal": round(p.y, 6),
                        "yaw_ideal": round(p.yaw, 6)})
        return out

    def write_csv(self, path: str, dt: float = DT_DEFAULT) -> None:
        rows = self.rows(dt)
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

    def describe(self) -> Dict[str, object]:
        return {"name": self.name, "params": dict(self.params),
                "duration_s": round(self.duration, 3),
                "limits": asdict(self.limits),
                "segments": [asdict(s) for s in self.segments]}


# ------------------------------------------------------------------- paths --

def _require(v: float, w: float, limits: Limits) -> None:
    """Reject a REQUESTED body speed that exceeds the limits (a short segment could
    otherwise hide it by never reaching the peak)."""
    if abs(v) > limits.v_max + 1e-9:
        raise ValueError(f"requested |v| {abs(v):.3f} > v_max {limits.v_max}")
    if abs(w) > limits.w_max + 1e-9:
        raise ValueError(f"requested |w| {abs(w):.3f} > w_max {limits.w_max}")
    wheel = abs(v) + abs(w) * limits.half_track
    if wheel > limits.wheel_v_max + 1e-9:
        raise ValueError(f"requested wheel speed {wheel:.3f} > {limits.wheel_v_max} m/s")


def _wrap(name: str, params: Dict[str, object], motion: Sequence[Segment], limits: Limits,
          pre_rest: float, post_rest: float, dwell: float) -> Profile:
    segs: List[Segment] = [rest(pre_rest, "rest_pre")]
    for i, s in enumerate(motion):
        if i > 0 and dwell > 0.0:
            segs.append(rest(dwell, f"dwell{i}"))
        segs.append(s)
    segs.append(rest(post_rest, "rest_post"))
    p = Profile(name, params, segs, limits)
    p.validate()
    return p


def straight(length: float, v: float, limits: Limits = Limits(),
             pre_rest: float = 3.0, post_rest: float = 3.0) -> Profile:
    """One straight line of `length` m at `v` m/s (negative length = backward)."""
    _require(v, 0.0, limits)
    seg = straight_segment(length, v, limits.accel, "straight")
    return _wrap("straight", {"length": length, "v": v}, [seg], limits, pre_rest, post_rest, 0.0)


def circle(radius: float, v: float, direction: str = "ccw", limits: Limits = Limits(),
           pre_rest: float = 3.0, post_rest: float = 3.0) -> Profile:
    """One full circle of `radius` m at `v` m/s. direction ccw (left) or cw (right)."""
    sign = 1.0 if direction == "ccw" else -1.0
    _require(v, v / radius, limits)
    seg = arc_segment(radius, sign * 2.0 * math.pi, v, limits.accel, "circle")
    return _wrap("circle", {"radius": radius, "v": v, "direction": direction},
                 [seg], limits, pre_rest, post_rest, 0.0)


def square(side: float, v: float, corner: str = "spot", turn_w: float = 0.5,
           corner_radius: float = 0.3, direction: str = "ccw", dwell: float = 1.0,
           limits: Limits = Limits(), pre_rest: float = 3.0, post_rest: float = 3.0) -> Profile:
    """Square of `side` m. corner='spot': 4 straights + 4 in-place 90 deg turns at `turn_w`;
    corner='arc': straights of side-2r joined by 90 deg arcs of radius r (no stop)."""
    sign = 1.0 if direction == "ccw" else -1.0
    motion: List[Segment] = []
    if corner == "spot":
        _require(v, 0.0, limits)
        _require(0.0, turn_w, limits)
        for i in range(4):
            motion.append(straight_segment(side, v, limits.accel, f"side{i + 1}"))
            motion.append(spot_segment(sign * math.pi / 2.0, turn_w, limits.ang_accel, f"turn{i + 1}"))
        d = dwell
    elif corner == "arc":
        run = side - 2.0 * corner_radius
        if run <= 0.0:
            raise ValueError("corner_radius too large for side")
        _require(v, v / corner_radius, limits)
        for i in range(4):
            motion.append(straight_segment(run, v, limits.accel, f"side{i + 1}"))
            motion.append(arc_segment(corner_radius, sign * math.pi / 2.0, v, limits.accel, f"corner{i + 1}"))
        d = dwell
    else:
        raise ValueError("corner must be 'spot' or 'arc'")
    return _wrap("square", {"side": side, "v": v, "corner": corner, "turn_w": turn_w,
                            "corner_radius": corner_radius, "direction": direction, "dwell": dwell},
                 motion, limits, pre_rest, post_rest, d)


def s_curve(radius: float, v: float, arc_deg: float = 180.0, first: str = "left",
            join: str = "continuous", dwell: float = 1.0, limits: Limits = Limits(),
            pre_rest: float = 3.0, post_rest: float = 3.0) -> Profile:
    """S: an arc of `arc_deg` to the `first` side, then the same arc the other way.
    join='continuous' keeps speed through the inflection; 'stop' ramps to zero between."""
    ang = math.radians(arc_deg)
    left = first == "left"
    _require(v, v / radius, limits)
    if join == "continuous":
        motion = [s_curve_segment(radius, ang, v, limits.accel, left, "s")]
        d = 0.0
    elif join == "stop":
        s1 = 1.0 if left else -1.0
        motion = [arc_segment(radius, s1 * ang, v, limits.accel, "arc1"),
                  arc_segment(radius, -s1 * ang, v, limits.accel, "arc2")]
        d = dwell
    else:
        raise ValueError("join must be 'continuous' or 'stop'")
    return _wrap("s_curve", {"radius": radius, "v": v, "arc_deg": arc_deg, "first": first,
                             "join": join, "dwell": dwell}, motion, limits, pre_rest, post_rest, d)


def spot(angle_deg: float = 360.0, turn_w: float = 0.5, limits: Limits = Limits(),
         pre_rest: float = 3.0, post_rest: float = 3.0) -> Profile:
    """Turn in place through `angle_deg` (+ = ccw/left) at `turn_w` rad/s."""
    _require(0.0, turn_w, limits)
    seg = spot_segment(math.radians(angle_deg), turn_w, limits.ang_accel, "spot")
    return _wrap("spot", {"angle_deg": angle_deg, "turn_w": turn_w}, [seg], limits,
                 pre_rest, post_rest, 0.0)


PATHS = {"straight": straight, "circle": circle, "square": square, "s_curve": s_curve,
         "spot": spot}


@dataclass
class Footprint:
    """Axis-aligned room space the run needs, in the robot's START frame (x forward, y left)."""
    xmin: float
    xmax: float
    ymin: float
    ymax: float

    @property
    def size(self) -> Tuple[float, float]:
        return self.xmax - self.xmin, self.ymax - self.ymin

    def fits(self, arena: float, margin: float) -> bool:
        """Fits with the path axes parallel to the walls."""
        dx, dy = self.size
        return dx <= arena - 2 * margin + 1e-9 and dy <= arena - 2 * margin + 1e-9

    def fits_diagonal(self, arena: float, margin: float) -> bool:
        """Fits when the start heading points along the room diagonal (45 deg)."""
        dx, dy = self.size
        return (dx + dy) / math.sqrt(2.0) <= arena - 2 * margin + 1e-9

    def placement(self, arena: float, margin: float) -> str:
        """'wall', 'diagonal' or 'none' (does not fit)."""
        if self.fits(arena, margin):
            return "wall"
        if self.fits_diagonal(arena, margin):
            return "diagonal"
        return "none"

    def start_offset(self, margin: float) -> Tuple[float, float]:
        """Where to put the robot centre, measured from the room corner it faces away
        from: (distance from the wall behind, distance from the wall on the right)."""
        return margin - self.xmin, margin - self.ymin


def footprint(profile: Profile, half_diag: float, dt: float = DT_DEFAULT) -> Footprint:
    """Ideal path swept by a circle of radius `half_diag` (robot body half-diagonal).
    Open-loop runs drift, which is what the arena margin is for."""
    poses = profile.ideal_path(dt)
    xs = [p.x for p in poses]
    ys = [p.y for p in poses]
    return Footprint(min(xs) - half_diag, max(xs) + half_diag,
                     min(ys) - half_diag, max(ys) + half_diag)


def build(name: str, **params) -> Profile:
    """Build a named path; unknown names raise KeyError."""
    return PATHS[name](**params)


# --------------------------------------------------------------------- cli --

def main(argv: Iterable[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="print or export an OROHA path profile")
    ap.add_argument("path", choices=sorted(PATHS))
    ap.add_argument("--v", type=float, default=0.2)
    ap.add_argument("--length", type=float, default=2.0)
    ap.add_argument("--radius", type=float, default=0.75)
    ap.add_argument("--side", type=float, default=1.0)
    ap.add_argument("--corner", default="spot")
    ap.add_argument("--turn-w", type=float, default=0.5)
    ap.add_argument("--direction", default="ccw")
    ap.add_argument("--arc-deg", type=float, default=180.0)
    ap.add_argument("--first", default="left")
    ap.add_argument("--join", default="continuous")
    ap.add_argument("--angle-deg", type=float, default=360.0)
    ap.add_argument("--half-diag", type=float, default=0.36, help="robot body half-diagonal [m]")
    ap.add_argument("--arena", type=float, default=3.0)
    ap.add_argument("--margin", type=float, default=0.3)
    ap.add_argument("--dt", type=float, default=DT_DEFAULT)
    ap.add_argument("--csv", help="write samples + ideal pose to this CSV")
    a = ap.parse_args(list(argv) if argv is not None else None)

    common = {"v": a.v}
    if a.path == "straight":
        p = straight(a.length, **common)
    elif a.path == "circle":
        p = circle(a.radius, direction=a.direction, **common)
    elif a.path == "square":
        p = square(a.side, corner=a.corner, turn_w=a.turn_w, direction=a.direction, **common)
    elif a.path == "spot":
        p = spot(a.angle_deg, turn_w=a.turn_w)
    else:
        p = s_curve(a.radius, arc_deg=a.arc_deg, first=a.first, join=a.join, **common)

    end = p.ideal_path(a.dt)[-1]
    print(f"{p.name} {p.params}")
    print(f"duration {p.duration:.2f} s, path length {p.path_length(a.dt):.3f} m, "
          f"end pose x={end.x:.3f} y={end.y:.3f} yaw={math.degrees(end.yaw):.1f} deg")
    fp = footprint(p, a.half_diag, a.dt)
    dx, dy = fp.size
    ox, oy = fp.start_offset(a.margin)
    place = fp.placement(a.arena, a.margin)
    msg = {"wall": f"fits along the walls; start {ox:.2f} m from the wall behind, {oy:.2f} m from the wall on the right",
           "diagonal": "fits only along the room diagonal (start near a corner, heading to the opposite corner)",
           "none": "DOES NOT FIT the room"}[place]
    print(f"room needed {dx:.2f} x {dy:.2f} m (arena {a.arena} m, margin {a.margin} m, half-diag {a.half_diag} m): {msg}")
    for i, s in enumerate(p.segments):
        print(f"  [{i}] {s.label:10s} {s.kind:8s} {s.duration:6.2f} s  v={s.v_peak:+.3f} w={s.w_peak:+.3f} ramp={s.ramp:.2f}")
    if a.csv:
        p.write_csv(a.csv, a.dt)
        print(f"wrote {a.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
