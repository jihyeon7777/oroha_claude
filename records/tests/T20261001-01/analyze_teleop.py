"""T20261001-01 teleop bag analysis: release latency, wheel stop, hard stop, directions, IMU gaps."""
import sys
sys.path.insert(0, "src/oroha_tools")
from oroha_tools.export_csv import read_bag

bag = sys.argv[1] if len(sys.argv) > 1 else "data/tests/T20261001-01/bag"
cmd, js, logs, imu = [], [], [], []
for topic, _, t_ns, m in read_bag(bag):
    if topic == "/diff_cont/cmd_vel":
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        cmd.append((s, m.twist.linear.x, m.twist.angular.z))
    elif topic == "/joint_states":
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        v = dict(zip(m.name, m.velocity))
        js.append((s, v["motor_L"], v["motor_R"]))
    elif topic == "/rosout" and m.name == "oroha_teleop":
        logs.append((m.stamp.sec + m.stamp.nanosec * 1e-9, m.msg))
    elif topic == "/imu/data":
        imu.append(t_ns * 1e-9)
t0 = cmd[0][0] if cmd else 0.0
R = 0.003635 * 1.0          # wheel_radius in yaml is per MOTOR rad (34.615:1 folded in)

def wheel_at(t):
    return min(js, key=lambda r: abs(r[0] - t))

print(f"cmd_vel {len(cmd)} msgs over {cmd[-1][0]-t0:.1f} s; joint_states {len(js)}; teleop logs {len(logs)}")
print("\n== release events (log -> cmd ramp -> wheel speed)")
for ts, msg in logs:
    if not msg.startswith("released"):
        continue
    after = [c for c in cmd if c[0] >= ts - 0.06]
    before = [c for c in cmd if c[0] < ts - 0.06]
    v0 = before[-1] if before else None
    # wheel |v| below 5 % of the pre-release value
    w0 = wheel_at(ts)
    peak = max(abs(w0[1]), abs(w0[2]))
    stop = next((r for r in js if r[0] > ts and max(abs(r[1]), abs(r[2])) < 0.05 * max(peak, 1e-6)), None)
    print(f"t={ts-t0:7.2f}  {msg[10:]:<55} cmd before v={v0[1]:+.3f} w={v0[2]:+.3f}" if v0 else msg,
          f"| wheel L {w0[1]:+6.1f} R {w0[2]:+6.1f} rad/s -> <5 % after {stop[0]-ts:.2f} s" if stop else "")
print("\n== hard stops (non-zero -> exact zero in one step)")
for a, b in zip(cmd, cmd[1:]):
    if (abs(a[1]) > 0.02 or abs(a[2]) > 0.05) and b[1] == 0.0 and b[2] == 0.0 and b[0] - a[0] < 0.2:
        w0 = wheel_at(b[0])
        stop = next((r for r in js if r[0] > b[0] and max(abs(r[1]), abs(r[2])) < 0.05 * max(abs(w0[1]), abs(w0[2]), 1e-6)), None)
        print(f"t={b[0]-t0:7.2f}  from v={a[1]:+.3f} w={a[2]:+.3f} -> 0 | wheel <5 % after {stop[0]-b[0]:.2f} s" if stop else "")
print("\n== direction samples (steady segments): sign of wheel L/R per command")
seen = {}
for s, v, w in cmd:
    key = ("v+" if v > 0.05 else "v-" if v < -0.05 else "") + ("w+" if w > 0.2 else "w-" if w < -0.2 else "")
    if key and key not in seen:
        r = wheel_at(s + 0.6)
        seen[key] = (s - t0, v, w, r[1], r[2])
for k, (t, v, w, l, rr) in seen.items():
    print(f"{k:5} t={t:6.1f} cmd v={v:+.3f} w={w:+.3f} -> wheel L {l:+6.1f} R {rr:+6.1f} rad/s")
vmax = max(abs(c[1]) for c in cmd); wmax = max(abs(c[2]) for c in cmd)
print(f"\nmax |v| {vmax:.3f} m/s, max |w| {wmax:.3f} rad/s")
d = [b - a for a, b in zip(imu, imu[1:])]
d.sort()
print(f"\n/imu/data {len(imu)} msgs, {len(imu)/(imu[-1]-imu[0]):.1f} Hz; dt median {d[len(d)//2]*1e3:.1f} ms, "
      f"p99 {d[int(len(d)*0.99)]*1e3:.1f} ms, max {d[-1]*1e3:.1f} ms; gaps>100 ms: {sum(x > 0.1 for x in d)}")
