"""ROS-free tests for the Pico line protocol and conversion."""

from oroha_power.protocol import (Calibration, OffsetFilter, convert, convert_abs, flag_names,
                                  parse_data_line, parse_kv, rail_corr_from_rest, true_zero_raw)

CAL = Calibration(calib_id="t", a_per_lsb=12.133e-3, scale_gp27=0.94289, scale_gp28=0.94289,
                  sign_gp27=1, sign_gp28=1, zero_gp27=2035.257, zero_gp28=2033.974,
                  v_per_lsb=8.913e-3, gp26_b_lsb=-18.7, quiet_a=0.080)


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


def test_rail_corr_matches_firmware_zero_model():
    # firmware reply on 2026-10-01 (preflight 'Z'): gp28=2035.318 gp27=2037.471 -> rail_corr=1.002571
    assert abs(CAL.quiet_lsb - 7.0) < 0.01                       # 80 mA = 7.0 LSB (firmware QUIET_GP2x)
    rc = rail_corr_from_rest(CAL, 2037.471, 2035.318)
    assert abs(rc - 1.002571) < 2e-5
    # calibration-day powered rest (true 0 A + 7 LSB) -> rail unchanged
    assert abs(rail_corr_from_rest(CAL, 2035.257 + 7.0, 2033.974 + 7.0) - 1.0) < 2e-5


def test_currents_stay_on_true_zero_after_rail_correction():
    """H3: a powered-rest frame reads the quiescent 80 mA in absolute terms, whatever the rail."""
    for rc in (1.0, 1.0026, 0.995):
        z27, z28 = true_zero_raw(CAL, rc)
        q = CAL.quiet_lsb / rc                                   # quiescent in raw LSB at this rail
        f = parse_data_line("D,0,0,32,3123.0,3121,3125,%.3f,0,0,%.3f,0,0,64" % (z27 + q, z28 + q))
        _, il, ir = convert_abs(f, CAL, rc)
        assert abs(il - 0.080) < 2e-3 and abs(ir - 0.080) < 2e-3
        # and the rail estimate from that same frame gives back rc
        assert abs(rail_corr_from_rest(CAL, f.gp27, f.gp28) - rc) < 1e-4


def test_offset_filter_min_and_restart():
    flt = OffsetFilter(window=5)
    # device 10.00, 10.02, ... arriving with jitter 10..30 ms: offset = min(rx - dev)
    for k, jitter in enumerate([0.030, 0.010, 0.025, 0.015, 0.020]):
        off, resid = flt.update(110.0 + k * 0.02 + jitter, 10.0 + k * 0.02)
    assert abs(off - 100.010) < 1e-9 and abs(resid - 0.010) < 1e-9
    # device reboot: t_us restarts near 0 (> 1 s backwards) -> filter resets, restart counted
    off, _ = flt.update(111.0, 0.0)
    assert flt.restarts == 1 and len(flt) == 1 and abs(off - 111.0) < 1e-9
