import sys, os, math, statistics as st, yaml, subprocess
sys.path[:0]=["src/oroha_tools","src/oroha_power"]
from oroha_tools.export_csv import read_bag
for r in sys.argv[1:]:
    if not os.path.exists(r+"/csv/summary.yaml"):
        subprocess.run(["oroha_export_csv", r], capture_output=True)
    s=yaml.safe_load(open(r+"/csv/summary.yaml")); meta=yaml.safe_load(open(r+"/meta.yaml")); t0=meta["start_ros_ns"]
    gz=[]
    for topic,_,t,m in read_bag(r+"/bag"):
        if topic=="/imu/data": gz.append((m.header.stamp.sec*1e9+m.header.stamp.nanosec, m.angular_velocity.z))
    bias=st.fmean([z for ts,z in gz if t0+0.3e9<=ts<=t0+2.8e9]); yaw=0; prev=None
    for ts,z in gz:
        if ts<t0: continue
        if prev: yaw+=(z-bias)*(ts-prev)*1e-9
        prev=ts
    o=s["odometry"] or {}; p=s["power"]; mi=o.get("odom_minus_ideal",{})
    print("%s %s %s | odom end-ideal x %+.3f y %+.3f yaw %+.1f deg | gyro yaw %+.1f deg (ideal %s) | E inc %.4f abs %.4f Wh | max I %.2f/%.2f A" % (
        os.path.basename(r), s["status"], meta["params"], mi.get("x",0), mi.get("y",0), math.degrees(mi.get("yaw_rad",0)), math.degrees(yaw),
        "%.0f"%math.degrees(meta["profile"].get("end_yaw_rad", 0)) if isinstance(meta.get("profile"),dict) and "end_yaw_rad" in meta["profile"] else "-", p["energy_wh"]["inc_total"], p["energy_wh"]["abs_total"], p["max_a"]["i_left_abs"], p["max_a"]["i_right_abs"]))
