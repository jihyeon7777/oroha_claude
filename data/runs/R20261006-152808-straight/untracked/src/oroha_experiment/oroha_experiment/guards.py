"""Per-wheel motion guard for the experiment runner (plan review M2).

The old guard looked at max(|wheel speed|) only, so one dead side (MD400 lost, wheel blocked,
belt off) passed as long as the other side turned. Here every side is checked against the
speed the command asks of it:

  expected motor-shaft speed  w_L = (v - w*sep/2) / r,  w_R = (v + w*sep/2) / r
  (diff_cont's wheel_radius r already folds the 34.615:1 reduction in: rad/s at the motor shaft)

  NOT_TURNING  |expected| >= min_expected and |measured| < still        for > timeout
  WRONG_WAY    |expected| >= min_expected and measured opposes expected  for > timeout

The command is the profile's (before diff_cont's acceleration limits); the timeout absorbs the
ramp lag (1.0 m/s^2 -> 0.35 m/s in 0.35 s). A verdict carries the evidence the runner adds
(Pico branch currents, MD400 status bits) so E-stop (drive power cut: currents fall to the
quiescent level) and a blocked wheel (current high) can be told apart.
"""

from __future__ import annotations

from dataclasses import dataclass, field

SIDES = ("left", "right")


@dataclass
class WheelGuard:
    wheel_radius: float = 0.003635     # m per motor rad (diff_cont wheel_radius)
    wheel_separation: float = 0.451
    min_expected: float = 10.0         # motor rad/s (~0.036 m/s): smaller demands are not checked
    still: float = 2.0                 # motor rad/s: below = not turning
    timeout: float = 1.0               # s
    _since: dict = field(default_factory=dict)

    def expected(self, v: float, w: float) -> tuple:
        h = 0.5 * w * self.wheel_separation
        return (v - h) / self.wheel_radius, (v + h) / self.wheel_radius

    def reset(self):
        self._since.clear()

    def update(self, t: float, v: float, w: float, meas_left: float, meas_right: float):
        """Feed one tick (t in s). Returns None or (kind, side, expected, measured, held_s)."""
        exp = self.expected(v, w)
        found = None
        for side, e, m in zip(SIDES, exp, (meas_left, meas_right)):
            for kind in ("NOT_TURNING", "WRONG_WAY"):
                key = (side, kind)
                if abs(e) < self.min_expected:
                    bad = False
                elif kind == "NOT_TURNING":
                    bad = abs(m) < self.still
                else:
                    bad = abs(m) >= self.still and m * e < 0
                if not bad:
                    self._since.pop(key, None)
                    continue
                t0 = self._since.setdefault(key, t)
                if t - t0 > self.timeout and found is None:
                    found = (kind, side, e, m, t - t0)
        return found


def describe(verdict, currents=None, status=None, quiet_a: float = 0.080) -> str:
    """Human/FAIL text for a verdict, with the evidence that separates the usual causes."""
    kind, side, e, m, held = verdict
    s = "%s wheel %s: expected %+.1f rad/s, measured %+.1f for %.1f s" % (
        side.upper(), "not turning" if kind == "NOT_TURNING" else "turning the WRONG WAY", e, m, held)
    hints = []
    if currents is not None:
        il, ir = currents
        s += "; Pico I L %.2f R %.2f A" % (il, ir)
        if kind == "NOT_TURNING":
            i_side = il if side == "left" else ir
            if max(il, ir) < quiet_a + 0.07:
                hints.append("both branches at quiescent -> drive power cut (E-stop?)")
            elif i_side > 0.6:
                hints.append("high current on that side -> blocked wheel / belt?")
            elif i_side < quiet_a + 0.07:
                hints.append("that branch idle -> MD400 not driving (comm lost / alarm?)")
    if status is not None:
        s += "; MD400 status L 0x%02x R 0x%02x" % status
        if any(status):
            hints.append("MD400 alarm bits set")
    if hints:
        s += " (" + "; ".join(hints) + ")"
    return s
