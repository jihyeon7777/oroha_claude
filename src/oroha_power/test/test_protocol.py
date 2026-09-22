"""ROS-free tests for the Pico line protocol and conversion."""

from oroha_power.protocol import (Calibration, OffsetFilter, convert, flag_names,
                                  parse_data_line, parse_kv)

CAL = Calibration(calib_id="t", a_per_lsb=12.133e-3, scale_gp27=0.94289, scale_gp28=0.94289,
                  sign_gp27=1, sign_gp28=1, zero_gp27=2035.257, zero_gp28=2033.974,
                  v_per_lsb=8.913e-3, gp26_b_lsb=-18.7)


def test_parse_data_line():
    f = parse_data_line("D,412,4120000,32,2607.45,2604,2610,2047.52,2043,2052,2047.48,2043,2052,64")
    assert f and f.seq == 412 and f.t_us == 4120000 and f.n == 32
    assert f.gp27 == 2047.52 and f.gp28_lo == 2043 and f.zero_valid and not f.overrun
    assert parse_data_line("D,1,2,3") is None
    assert parse_data_line("E,1,2,3,4,5,6,7,8,9,10,11,12,13") is None


def test_flags_and_kv():
    assert flag_names(0xC0) == ["zero_valid", "overrun"]
    kv = parse_kv("#ZERO gp28=2034.0 gp27=2035.3 rail=3.2887 rail_corr=1.0000 n=1024")
    assert kv["gp28"] == "2034.0" and kv["rail_corr"] == "1.0000"
    assert parse_kv("#CFG fw=oroha-bench-1.1 smps_pwm=1")["fw"] == "oroha-bench-1.1"


def test_convert_matches_documented_formulas():
    f = parse_data_line("D,0,0,32,3123.0,3121,3125,2122.7,2120,2125,2121.4,2119,2124,64")
    v, il, ir = convert(f, CAL, CAL.zero_gp27, CAL.zero_gp28, 1.0)
    assert abs(v - (3123.0 + 18.7) * 8.913e-3) < 1e-9          # V_bus = (raw + 18.7) x 8.913 mV
    assert abs(il - (2122.7 - 2035.257) * 11.44e-3) < 1e-4       # 11.44 mA/LSB
    assert abs(ir - (2121.4 - 2033.974) * 11.44e-3) < 1e-4
    assert il > 0 and ir > 0                                     # discharge positive


def test_offset_filter_min_and_restart():
    flt = OffsetFilter(window=5)
    # device 10.00, 10.02, ... arriving with jitter 10..30 ms: offset = min(rx - dev)
    for k, jitter in enumerate([0.030, 0.010, 0.025, 0.015, 0.020]):
        off, resid = flt.update(110.0 + k * 0.02 + jitter, 10.0 + k * 0.02)
    assert abs(off - 100.010) < 1e-9 and abs(resid - 0.010) < 1e-9
    # device reboot: t_us restarts near 0 (> 1 s backwards) -> filter resets, restart counted
    off, _ = flt.update(111.0, 0.0)
    assert flt.restarts == 1 and len(flt) == 1 and abs(off - 111.0) < 1e-9
