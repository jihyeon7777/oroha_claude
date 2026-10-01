"""Pico firmware (oroha-bench-1.x) line protocol — ROS-free, unit-testable.

Stream header:  #OROHA oroha-bench-1.1  rate=50 n=32
                #COL seq,t_us,n,v_mean,v_min,v_max,gp27_mean,gp27_min,gp27_max,gp28_mean,gp28_min,gp28_max,flags
Data line:      D,<14 fields>  — all raw 12-bit ADC; means fractional, min/max integer.
Meta lines:     #CFG fw=... / #ZERO gp28=.. gp27=.. rail=.. rail_corr=.. n=.. / #STAT / #STOP / #ERR / #WARN
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

FLAG_V_UNDER, FLAG_GP27_UNDER, FLAG_GP28_UNDER = 0x01, 0x02, 0x04
FLAG_V_OVER, FLAG_GP27_OVER, FLAG_GP28_OVER = 0x08, 0x10, 0x20
FLAG_ZERO_VALID = 0x40
FLAG_OVERRUN = 0x80
FLAG_NAMES = {0: "v_under", 1: "gp27_under", 2: "gp28_under",
              3: "v_over", 4: "gp27_over", 5: "gp28_over",
              6: "zero_valid", 7: "overrun"}
FLAG_BAD_MASK = 0x3F | FLAG_OVERRUN


@dataclass
class Frame:
    seq: int
    t_us: int
    n: int
    v: float
    v_lo: int
    v_hi: int
    gp27: float
    gp27_lo: int
    gp27_hi: int
    gp28: float
    gp28_lo: int
    gp28_hi: int
    flags: int

    @property
    def zero_valid(self) -> bool:
        return bool(self.flags & FLAG_ZERO_VALID)

    @property
    def overrun(self) -> bool:
        return bool(self.flags & FLAG_OVERRUN)


def parse_data_line(line: str) -> Optional[Frame]:
    """'D,...' with exactly 14 fields -> Frame, else None."""
    p = line.strip().split(",")
    if len(p) != 14 or p[0] != "D":
        return None
    try:
        return Frame(seq=int(p[1]), t_us=int(p[2]), n=int(p[3]),
                     v=float(p[4]), v_lo=int(p[5]), v_hi=int(p[6]),
                     gp27=float(p[7]), gp27_lo=int(p[8]), gp27_hi=int(p[9]),
                     gp28=float(p[10]), gp28_lo=int(p[11]), gp28_hi=int(p[12]),
                     flags=int(p[13]))
    except ValueError:
        return None


def parse_kv(line: str) -> Dict[str, str]:
    """'#ZERO gp28=2034.1 gp27=2035.3 rail=3.2887 rail_corr=1.0000 n=1024' -> dict."""
    out: Dict[str, str] = {}
    for tok in line.split()[1:]:
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def flag_names(flags: int) -> list:
    return [FLAG_NAMES[i] for i in range(8) if flags & (1 << i)]


@dataclass(frozen=True)
class Calibration:
    """Host-side conversion constants (see config/calibration/*.yaml)."""
    calib_id: str
    a_per_lsb: float
    scale_gp27: float
    scale_gp28: float
    sign_gp27: float
    sign_gp28: float
    zero_gp27: float
    zero_gp28: float
    v_per_lsb: float
    gp26_b_lsb: float
    scale_v: float = 1.0
    quiet_a: float = 0.0
    valid_range_a: tuple = (0.0, 1.2)

    def k(self, ch: str) -> float:
        """Signed A/LSB of channel 'gp27' or 'gp28' at the calibration rail."""
        return self.a_per_lsb * getattr(self, "scale_" + ch) * getattr(self, "sign_" + ch)

    @property
    def quiet_lsb(self) -> float:
        """Controller quiescent current at powered rest, in LSB (firmware QUIET_GP2x = 7.0)."""
        return self.quiet_a / (self.a_per_lsb * 0.5 * (self.scale_gp27 + self.scale_gp28))

    @classmethod
    def from_dict(cls, d: dict) -> "Calibration":
        c, v = d["current"], d["voltage"]
        return cls(calib_id=str(d["calib_id"]),
                   a_per_lsb=float(c["a_per_lsb"]),
                   scale_gp27=float(c["scale_gp27"]), scale_gp28=float(c["scale_gp28"]),
                   sign_gp27=float(c["sign_gp27"]), sign_gp28=float(c["sign_gp28"]),
                   zero_gp27=float(c["zero_gp27"]), zero_gp28=float(c["zero_gp28"]),
                   v_per_lsb=float(v["v_per_lsb"]), gp26_b_lsb=float(v["gp26_b_lsb"]),
                   scale_v=float(v.get("scale_v", 1.0)), quiet_a=float(c.get("quiet_a", 0.0)),
                   valid_range_a=tuple(float(x) for x in c.get("valid_range_a", (0.0, 1.2))))


def convert(f: Frame, cal: Calibration, zero_gp27: float, zero_gp28: float,
            rail_corr: float) -> tuple:
    """(v_bus [V], i_left = GP27 [A], i_right = GP28 [A])."""
    v = (f.v - cal.gp26_b_lsb) * cal.v_per_lsb * cal.scale_v * rail_corr
    i_left = (f.gp27 - zero_gp27) * cal.a_per_lsb * cal.scale_gp27 * cal.sign_gp27 * rail_corr
    i_right = (f.gp28 - zero_gp28) * cal.a_per_lsb * cal.scale_gp28 * cal.sign_gp28 * rail_corr
    return v, i_left, i_right


def rail_corr_from_rest(cal: Calibration, rest_gp27: float, rest_gp28: float) -> float:
    """Rail ratio (now / calibration) from a powered-rest raw pair — the firmware's '#ZERO' model.

    The ACS37030 is non-ratiometric: its 0 A output is a fixed voltage, so the raw 0 A point
    scales as 1/rail. Powered-rest raw = true 0 A raw + controller quiescent (quiet_lsb), hence
    rail_corr = mean(calibration 0 A raw) / mean(rest raw - quiet). It assumes the quiescent
    current is the calibrated 80 mA; it cannot tell a rail change from a sensor-offset drift.
    """
    q = cal.quiet_lsb
    z = 0.5 * ((rest_gp27 - q) + (rest_gp28 - q))
    return 0.5 * (cal.zero_gp27 + cal.zero_gp28) / z


def true_zero_raw(cal: Calibration, rail_corr: float) -> tuple:
    """(gp27, gp28) raw at TRUE 0 A for a given rail_corr."""
    return cal.zero_gp27 / rail_corr, cal.zero_gp28 / rail_corr


def convert_abs(f: Frame, cal: Calibration, rail_corr: float) -> tuple:
    """(v_bus, i_left, i_right) with currents relative to TRUE 0 A (not the powered-rest baseline)."""
    z27, z28 = true_zero_raw(cal, rail_corr)
    return convert(f, cal, z27, z28, rail_corr)


class OffsetFilter:
    """NTP-style minimum filter: offset = min(rx_time - device_time) over a window.

    Host arrival times are late by a variable USB/CDC delay (bursts of 6-7 lines,
    ~65 ms smear), never early, so the minimum tracks the true offset.
    """

    def __init__(self, window: int = 500):
        self.window = max(2, int(window))
        self._buf: list = []
        self.restarts = 0
        self._last_dev = None

    def update(self, rx_s: float, dev_s: float) -> tuple:
        """Feed one sample -> (offset_s, residual_s). Resets when device time goes backwards."""
        if self._last_dev is not None and dev_s < self._last_dev - 1.0:
            self._buf.clear()
            self.restarts += 1
        self._last_dev = dev_s
        d = rx_s - dev_s
        self._buf.append(d)
        if len(self._buf) > self.window:
            del self._buf[0]
        off = min(self._buf)
        return off, d - off

    def __len__(self) -> int:
        return len(self._buf)
