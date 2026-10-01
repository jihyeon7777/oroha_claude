"""T20261001-03 (B2) analysis: torque-off vs enabled rest, '~/zero', speed steps, Pico vs MD400 current.

  python3 records/tests/T20261001-03/analyze_b2.py data/tests/T20261001-03/bag <phases.log>
"""
import statistics as st
import sys

sys.path.insert(0, "src/oroha_tools")
sys.path.insert(0, "src/oroha_power")
from oroha_tools.export_csv import read_bag                       # noqa: E402
from oroha_tools.power_analysis import analyse, load_calibration  # noqa: E402

bag, phlog = sys.argv[1], sys.argv[2]
ph = {}
for line in open(phlog):
    p = line.split()
    if len(p) == 2 and p[0].replace("_", "").replace(".", "").replace("-", "").isalnum():
        try:
            ph[p[0]] = float(p[1])
        except ValueError:
            pass
T0 = ph["torque_off_rest_start"]
rel = {k: v - T0 for k, v in ph.items()}

prows, js, cmd, events = [], [], [], []
for topic, _, t_ns, m in read_bag(bag):
    if topic == "/oroha_power/sample":
        dev = m.device_stamp.sec + m.device_stamp.nanosec * 1e-9
        prows.append({"t_dev": dev - T0, "dev_ns": int(dev * 1e9), "gp26_mean": m.gp26_mean,
                      "gp27_mean": m.gp27_mean, "gp28_mean": m.gp28_mean, "i_left": m.i_left,
                      "i_right": m.i_right, "p_total": m.p_total, "v_bus": m.v_bus,
                      "rail_corr_node": m.rail_corr, "calib_id": m.calib_id})
    elif topic == "/joint_states":
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - T0
        v = dict(zip(m.name, m.velocity))
        e = dict(zip(m.name, m.effort))
        js.append((s, v["motor_L"], v["motor_R"], e["motor_L"], e["motor_R"]))
    elif topic == "/diff_cont/cmd_vel_out":
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - T0
        cmd.append((s, m.twist.linear.x, m.twist.angular.z))
    elif topic == "/oroha_power/calibration_event":
        events.append(m.data)

cal = load_calibration(prows[0]["calib_id"])


def sel(a, b, rows=prows):
    return [r for r in rows if a <= r["t_dev"] <= b]


def mean(rows, k):
    return st.fmean(r[k] for r in rows)


print("== calibration events")
for e in events:
    print("  ", e.split(" ", 1)[1][:160])

off = sel(rel["torque_off_rest_start"] + 1, rel["torque_off_rest_end"] - 0.5)
en = sel(rel["enabled_rest_start"] + 1, rel["enabled_rest_end"] - 0.5)
print("\n== rest raw (LSB)          gp27(L)   gp28(R)   n")
print(f"   torque off           {mean(off,'gp27_mean'):9.3f} {mean(off,'gp28_mean'):9.3f} {len(off):4d}")
print(f"   enabled (0 rpm)      {mean(en,'gp27_mean'):9.3f} {mean(en,'gp28_mean'):9.3f} {len(en):4d}")
print(f"   enabled - off        {mean(en,'gp27_mean')-mean(off,'gp27_mean'):+9.3f} "
      f"{mean(en,'gp28_mean')-mean(off,'gp28_mean'):+9.3f}  (x {cal.k('gp27')*1e3:.2f} mA/LSB)")

# reference windows: enabled rest and the final rest after the last step
last_end = max(v for k, v in rel.items() if k.endswith("_end") and k.startswith("step"))
windows = [("enabled_rest", rel["enabled_rest_start"] + 1, rel["enabled_rest_end"] - 0.5),
           ("final_rest", last_end + 2.0, rel["final_rest_end"] - 0.3)]
summ = analyse(prows, cal, windows)
print("\n== run reference:", [(w["label"], w["raw_gp27"], w["raw_gp28"], w["rail_corr"]) for w in summ["current_reference"]["windows"]],
      "drift", summ["current_reference"]["baseline_drift_lsb"])

# node live value at enabled rest after the '~/zero' (should read ~ +0.080 A)
z_end = rel["enabled_rest_end"] + 2.5
first_step = rel["step_v0.2_start"]
nz = sel(z_end, first_step - 0.2)
if nz:
    print(f"   node after ~/zero at rest: i_left {mean(nz,'i_left'):+.4f} i_right {mean(nz,'i_right'):+.4f} A "
          f"(rail_corr {nz[-1]['rail_corr_node']:.6f})")
pre = sel(rel["enabled_rest_start"] + 1, rel["enabled_rest_end"] - 0.5)
print(f"   node before ~/zero at rest: i_left {mean(pre,'i_left'):+.4f} i_right {mean(pre,'i_right'):+.4f} A "
      f"(rail_corr {pre[-1]['rail_corr_node']:.6f})")


def plateau(target_v, target_w):
    """last 3 s of the longest run of cmd_vel_out == target."""
    on = [c for c in cmd if abs(c[1] - target_v) < 1e-3 and abs(c[2] - target_w) < 1e-3]
    if not on:
        return None
    runs, cur = [], [on[0]]
    for a, b in zip(on, on[1:]):
        if b[0] - a[0] < 0.25:
            cur.append(b)
        else:
            runs.append(cur)
            cur = [b]
    runs.append(cur)
    r = max(runs, key=len)
    return max(r[0][0], r[-1][0] - 3.0), r[-1][0]


print("\n== plateaus (last 3 s)   wheel L/R rad/s     Pico di L/R A      Pico abs L/R A    MD400 effort L/R A   V")
steps = [(0.2, 0.0), (0.4, 0.0), (0.6, 0.0), (0.76, 0.0), (-0.4, 0.0), (0.0, 2.0), (0.0, -2.0)]
for tv, tw in steps:
    p = plateau(tv, tw)
    if not p:
        print(f"   v={tv:+.2f} w={tw:+.1f}: no plateau")
        continue
    a, b = p
    pr = sel(a, b)
    jr = [j for j in js if a <= j[0] <= b]
    wl, wr = st.fmean(j[1] for j in jr), st.fmean(j[2] for j in jr)
    el, er = st.fmean(j[3] for j in jr), st.fmean(j[4] for j in jr)
    print(f"   v={tv:+.2f} w={tw:+.1f}  {wl:+7.1f} {wr:+7.1f}   {mean(pr,'di_left'):+.4f} {mean(pr,'di_right'):+.4f}   "
          f"{mean(pr,'i_left_abs'):+.4f} {mean(pr,'i_right_abs'):+.4f}   {el:5.2f} {er:5.2f}   {mean(pr,'v_bus_run'):.2f}")

print("\n== decel transients (min di within 2 s after each step's command end)")
for k, v in sorted(rel.items(), key=lambda kv: kv[1]):
    if k.startswith("step") and k.endswith("_end"):
        pr = sel(v - 0.5, v + 2.0)
        if pr:
            print(f"   {k:16} min di L {min(r['di_left'] for r in pr):+.3f}  R {min(r['di_right'] for r in pr):+.3f} A;"
                  f"  min v_bus {min(r['v_bus_run'] for r in pr):.2f} max {max(r['v_bus_run'] for r in pr):.2f} V")

print("\n== energy / range:", summ["energy_wh"], summ["range"]["frac_above"], summ["range"]["frac_below"])
