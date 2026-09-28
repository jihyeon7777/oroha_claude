"""Export a run's rosbag (mcap) to CSV files with a columns reference and a summary.

  oroha_export_csv data/runs/R20260925-143012-square [--out csv]

Writes <run>/csv/{cmd_vel,odom,joint_states,power,battery,imu,mag,events,status,diagnostics}.csv,
<run>/csv/columns.md (units, time bases) and <run>/csv/summary.yaml (durations, odometry vs
ideal, currents/energy, sample counts, calibration ids). Raw data are never modified;
processed columns state their method here and in columns.md.

Time columns: t = seconds since the START event (negative before it), ros_t_ns = bag
receive time, stamp_ns = message header stamp, t_us/device_stamp_ns = Pico device time.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import yaml

TOPIC_FILES = {
    "/diff_cont/cmd_vel": "cmd_vel", "/diff_cont/odom": "odom", "/joint_states": "joint_states",
    "/oroha_power/sample": "power", "/oroha_power/battery": "battery", "/imu/data": "imu",
    "/imu/mag": "mag", "/imu/temperature": "temperature", "/oroha_experiment/event": "events",
    "/oroha_experiment/status": "status", "/diagnostics": "diagnostics",
    "/oroha_power/calibration_event": "calibration_events",
}


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def odom_summary(samples, ideal_end=None, gt=None) -> dict:
    """samples: [(x, y, yaw)] in the odom frame, in time order (yaw wrapped, as published).

    Returns the path length, the end pose RELATIVE TO THE START POSE (rotated into the
    start heading: x forward, y left) and the unwrapped heading change, plus errors
    against the ideal end pose of the profile and the manual ground truth if given.
    """
    x0, y0, yaw0 = samples[0]
    length = 0.0
    yaw_acc = 0.0
    for (xa, ya, ta), (xb, yb, tb) in zip(samples, samples[1:]):
        length += math.hypot(xb - xa, yb - ya)
        yaw_acc += wrap_pi(tb - ta)
    dx, dy = samples[-1][0] - x0, samples[-1][1] - y0
    c, s = math.cos(yaw0), math.sin(yaw0)
    rel = {"x": c * dx + s * dy, "y": -s * dx + c * dy, "yaw_change_rad": yaw_acc}
    out = {"method": "diff_cont/odom, no-slip wheel odometry (motor-shaft counts, effective wheel_radius)",
           "path_length_m": round(length, 4),
           "end_pose_rel": {k: round(v, 4) for k, v in rel.items()}}
    if ideal_end:
        out["ideal_end"] = ideal_end
        out["odom_minus_ideal"] = {"x": round(rel["x"] - ideal_end["x"], 4),
                                   "y": round(rel["y"] - ideal_end["y"], 4),
                                   "yaw_rad": round(rel["yaw_change_rad"] - ideal_end["yaw"], 4)}
    if gt:
        out["manual_gt"] = gt
        out["odom_minus_gt"] = {"x": round(rel["x"] - gt["x_m"], 4), "y": round(rel["y"] - gt["y_m"], 4)}
        if ideal_end:
            out["ideal_minus_gt"] = {"x": round(ideal_end["x"] - gt["x_m"], 4),
                                     "y": round(ideal_end["y"] - gt["y_m"], 4)}
    return out


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _stamp_ns(msg) -> int | None:
    h = getattr(msg, "header", None)
    if h is None:
        return None
    return int(h.stamp.sec) * 1_000_000_000 + int(h.stamp.nanosec)


def read_bag(bag_dir: Path):
    """Yield (topic, type_name, ros_t_ns, msg) for every message, in time order."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        tname = types.get(topic)
        if tname is None:
            continue
        if tname not in classes:
            try:
                classes[tname] = get_message(tname)
            except Exception:  # noqa: BLE001
                classes[tname] = None
        cls = classes[tname]
        if cls is None:
            continue
        try:
            yield topic, tname, t_ns, deserialize_message(data, cls)
        except Exception:  # noqa: BLE001
            continue


class Writers:
    def __init__(self, out: Path):
        self.out = out
        self.files, self.writers, self.counts = {}, {}, {}

    def row(self, name: str, row: dict):
        if name not in self.writers:
            f = open(self.out / f"{name}.csv", "w", newline="")
            w = csv.DictWriter(f, fieldnames=list(row.keys()))
            w.writeheader()
            self.files[name], self.writers[name], self.counts[name] = f, w, 0
        self.writers[name].writerow(row)
        self.counts[name] += 1

    def close(self):
        for f in self.files.values():
            f.close()


def export(run_dir: Path, out: Path) -> dict:
    bag_dir = run_dir / "bag"
    if not bag_dir.exists():
        raise FileNotFoundError(f"{bag_dir} missing")
    if not (bag_dir / "metadata.yaml").exists():
        raise RuntimeError(f"{bag_dir}/metadata.yaml missing: the bag is still being recorded or the "
                           "recorder was killed before finalizing (run `ros2 bag reindex` to recover)")
    meta = yaml.safe_load(open(run_dir / "meta.yaml")) if (run_dir / "meta.yaml").exists() else {}
    out.mkdir(parents=True, exist_ok=True)

    # pass 1: find START (bag receive time) so t is run-relative
    t_start_ns = None
    first_ns = None
    for topic, tname, t_ns, msg in read_bag(bag_dir):
        first_ns = first_ns if first_ns is not None else t_ns
        if topic == "/oroha_experiment/event" and msg.event == "START":
            t_start_ns = t_ns
            break
    t0 = t_start_ns if t_start_ns is not None else (first_ns or 0)

    w = Writers(out)
    odom_samples = []
    power_prev_dev = None
    energy_wh = 0.0
    gap_count = 0
    gap_time = 0.0
    n_power = n_overrun = 0
    seq_prev = None
    seq_gaps = 0
    i_left_max = i_right_max = 0.0
    i_left_sum = i_right_sum = 0.0
    calib_ids = set()
    imu_yaw_first = imu_yaw_last = None
    gyro_int = 0.0
    gyro_prev = None
    joint_names = None

    for topic, tname, t_ns, msg in read_bag(bag_dir):
        t = (t_ns - t0) * 1e-9
        s_ns = _stamp_ns(msg)
        base = {"t": round(t, 6), "ros_t_ns": t_ns, "stamp_ns": s_ns}
        name = TOPIC_FILES.get(topic)
        if name is None:
            continue
        if name == "cmd_vel":
            w.row(name, {**base, "v": msg.twist.linear.x, "w": msg.twist.angular.z})
        elif name == "odom":
            p = msg.pose.pose.position
            yaw = _yaw(msg.pose.pose.orientation)
            if t >= 0.0 or t_start_ns is None:        # from START on (ARMED zeros excluded)
                odom_samples.append((p.x, p.y, yaw))
            w.row(name, {**base, "x": p.x, "y": p.y, "yaw": yaw,
                         "vx": msg.twist.twist.linear.x, "wz": msg.twist.twist.angular.z,
                         "frame_id": msg.header.frame_id, "child_frame_id": msg.child_frame_id})
        elif name == "joint_states":
            joint_names = joint_names or list(msg.name)
            row = dict(base)
            for i, jn in enumerate(msg.name):
                row[f"{jn}_pos"] = msg.position[i] if i < len(msg.position) else ""
                row[f"{jn}_vel"] = msg.velocity[i] if i < len(msg.velocity) else ""
                row[f"{jn}_eff"] = msg.effort[i] if i < len(msg.effort) else ""
            w.row(name, row)
        elif name == "power":
            dev_ns = int(msg.device_stamp.sec) * 1_000_000_000 + int(msg.device_stamp.nanosec)
            n_power += 1
            n_overrun += int(msg.overrun)
            if seq_prev is not None and msg.seq > seq_prev + 1:
                seq_gaps += msg.seq - seq_prev - 1
            seq_prev = msg.seq
            calib_ids.add(msg.calib_id)
            i_left_max, i_right_max = max(i_left_max, msg.i_left), max(i_right_max, msg.i_right)
            i_left_sum += msg.i_left
            i_right_sum += msg.i_right
            if power_prev_dev is not None:
                dt = (dev_ns - power_prev_dev) * 1e-9
                if 0.0 < dt <= 0.060:
                    energy_wh += msg.p_total * dt / 3600.0
                else:
                    gap_count += 1
                    gap_time += max(dt, 0.0)
            power_prev_dev = dev_ns
            w.row(name, {**base, "device_stamp_ns": dev_ns, "seq": msg.seq, "t_us": msg.t_us,
                         "n_rounds": msg.n_rounds,
                         "gp26_mean": msg.gp26_mean, "gp26_min": msg.gp26_min, "gp26_max": msg.gp26_max,
                         "gp27_mean": msg.gp27_mean, "gp27_min": msg.gp27_min, "gp27_max": msg.gp27_max,
                         "gp28_mean": msg.gp28_mean, "gp28_min": msg.gp28_min, "gp28_max": msg.gp28_max,
                         "flags": msg.flags, "zero_valid": int(msg.zero_valid), "overrun": int(msg.overrun),
                         "v_bus": msg.v_bus, "i_left": msg.i_left, "i_right": msg.i_right,
                         "p_left": msg.p_left, "p_right": msg.p_right, "p_total": msg.p_total,
                         "zero_gp27": msg.zero_gp27, "zero_gp28": msg.zero_gp28, "rail_corr": msg.rail_corr,
                         "sync_offset_s": msg.sync_offset_s, "sync_residual_ms": msg.sync_residual_ms,
                         "calib_id": msg.calib_id, "fw_version": msg.fw_version})
        elif name == "battery":
            w.row(name, {**base, "voltage": msg.voltage, "current": msg.current})
        elif name == "imu":
            yaw = _yaw(msg.orientation)
            imu_yaw_first = imu_yaw_first if imu_yaw_first is not None else yaw
            imu_yaw_last = yaw
            if gyro_prev is not None and s_ns:
                gyro_int += msg.angular_velocity.z * (s_ns - gyro_prev) * 1e-9
            gyro_prev = s_ns
            q, g, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
            w.row(name, {**base, "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w, "yaw": yaw,
                         "gx": g.x, "gy": g.y, "gz": g.z, "ax": a.x, "ay": a.y, "az": a.z,
                         "frame_id": msg.header.frame_id})
        elif name == "mag":
            m = msg.magnetic_field
            w.row(name, {**base, "mx": m.x, "my": m.y, "mz": m.z})
        elif name == "temperature":
            w.row(name, {**base, "temperature_c": msg.temperature})
        elif name == "events":
            w.row(name, {**base, "run_id": msg.run_id, "event": msg.event, "detail": msg.detail,
                         "repeat_index": msg.repeat_index, "segment_index": msg.segment_index,
                         "t_run": msg.t_run})
        elif name == "status":
            w.row(name, {**base, "state": msg.state, "t_run": msg.t_run, "segment_index": msg.segment_index,
                         "segment_label": msg.segment_label, "v_cmd": msg.v_cmd, "w_cmd": msg.w_cmd,
                         "bag_active": int(msg.bag_active)})
        elif name == "diagnostics":
            for st in msg.status:
                w.row(name, {**base, "name": st.name, "level": int.from_bytes(st.level, "little") if isinstance(st.level, bytes) else int(st.level),
                             "message": st.message,
                             "values": json.dumps({kv.key: kv.value for kv in st.values}, ensure_ascii=False)})
        elif name == "calibration_events":
            w.row(name, {**base, "data": msg.data})
    w.close()

    gt = None
    if (run_dir / "manual_gt.yaml").exists():
        gt = yaml.safe_load(open(run_dir / "manual_gt.yaml"))
    ideal_end = None
    prof = run_dir / "profile.csv"
    if prof.exists():
        rows = list(csv.DictReader(open(prof)))
        if rows:
            r = rows[-1]
            ideal_end = {"x": float(r["x_ideal"]), "y": float(r["y_ideal"]), "yaw": float(r["yaw_ideal"])}
    summary = {
        "run_id": meta.get("run_id", run_dir.name),
        "status": meta.get("status"),
        "t_start_found": t_start_ns is not None,
        "message_counts": w.counts,
        "odometry": None if len(odom_samples) < 2 else odom_summary(odom_samples, ideal_end, gt),
        "power": None if n_power == 0 else {
            "samples": n_power, "overrun_samples": n_overrun, "seq_gaps": seq_gaps,
            "energy_wh": round(energy_wh, 5),
            "energy_method": "sum(p_total*dt) on device_stamp; intervals > 60 ms excluded",
            "excluded_intervals": gap_count, "excluded_time_s": round(gap_time, 3),
            "i_left_mean_a": round(i_left_sum / n_power, 4), "i_right_mean_a": round(i_right_sum / n_power, 4),
            "i_left_max_a": round(i_left_max, 4), "i_right_max_a": round(i_right_max, 4),
            "calib_ids": sorted(calib_ids),
        },
        "imu": None if imu_yaw_first is None else {
            "fused_yaw_change_rad": round(imu_yaw_last - imu_yaw_first, 4),
            "gyro_integrated_yaw_rad": round(gyro_int, 4),
            "note": "fused yaw uses the magnetometer (unreliable indoors); gyro integral drifts ~-3 deg/min before zero_gyros",
        },
        "conditions": meta.get("conditions"),
        "versions": meta.get("versions"),
    }
    with open(out / "summary.yaml", "w") as f:
        yaml.safe_dump(summary, f, allow_unicode=True, sort_keys=False)
    (out / "columns.md").write_text(COLUMNS_MD)
    return summary


COLUMNS_MD = """# CSV columns

Time bases (all rows): `t` = seconds since the run's START event (bag receive time; negative
before START; falls back to the first message when no START exists), `ros_t_ns` = bag receive
time (ROS clock, ns), `stamp_ns` = message header stamp (ns; publisher's clock — for
oroha_power the host receive time, for um7 the receive time, for controllers the update time).

| file | columns | units / notes |
|---|---|---|
| cmd_vel.csv | v, w | m/s, rad/s commanded body velocity (TwistStamped on /diff_cont/cmd_vel) |
| odom.csv | x, y, yaw, vx, wz | m, m, rad (odom frame), m/s, rad/s — wheel odometry, no-slip assumption |
| joint_states.csv | <joint>_pos, _vel, _eff | rad, rad/s at the MOTOR SHAFT (34.615:1 to the wheel), A (MD400 internal, unsigned) |
| power.csv | device_stamp_ns, seq, t_us, n_rounds, gp2x_mean/min/max, flags, zero_valid, overrun | Pico window (20 ms): raw 12-bit ADC; t_us = device monotonic us; device_stamp_ns = t_us mapped to ROS time |
| power.csv | v_bus, i_left, i_right, p_left, p_right, p_total | V, A (discharge positive; LEFT=GP27=id2, RIGHT=GP28=id1), W; converted with calib_id |
| power.csv | zero_gp27, zero_gp28, rail_corr, sync_offset_s, sync_residual_ms, calib_id, fw_version | conversion inputs actually applied |
| battery.csv | voltage, current | V, A (ROS convention: discharge negative) |
| imu.csv | qx..qw, yaw, gx, gy, gz, ax, ay, az | ENU orientation, rad, rad/s, m/s^2 (um7_driver) |
| mag.csv | mx, my, mz | unit-norm direction (NOT tesla) |
| events.csv | event, detail, segment_index, t_run | runner events; t_run = runner's own run clock |
| status.csv | state, t_run, segment_*, v_cmd, w_cmd | 5 Hz runner status |
| diagnostics.csv | name, level, message, values(json) | 0 OK, 1 WARN, 2 ERROR |

summary.yaml states the method for every derived number (odometry path length, energy
integration window, IMU yaw). Raw bag files are the primary record; CSVs are exports.
"""


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run_dir")
    ap.add_argument("--out", help="output directory (default <run_dir>/csv)")
    a = ap.parse_args(argv)
    run_dir = Path(a.run_dir)
    out = Path(a.out) if a.out else run_dir / "csv"
    s = export(run_dir, out)
    print(yaml.safe_dump({"run_id": s["run_id"], "status": s["status"], "message_counts": s["message_counts"],
                          "odometry": s["odometry"], "power": s["power"]}, allow_unicode=True, sort_keys=False))
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
