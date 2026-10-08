#!/usr/bin/env bash
# Keyboard driving in ONE terminal: robot launch in the background + dead-man teleop in front.
#
#   bash setup/drive.sh                 # launch (motors + Pico) -> teleop; quit with x or Ctrl-C
#   bash setup/drive.sh --record        # + rosbag of the drive in data/drive/<time>/bag
#   bash setup/drive.sh --imu           # + UM7
#
# Keys (hold to drive, release = ramped stop): w/s forward/back, a/d turn in place, q/e forward
# while turning (radius 0.6 m), space/Esc stop, +/- speed scale, x quit. Speeds 0.15 m/s and
# 0.5 rad/s at the start (scale 0.5); max 0.3 m/s, 1.0 rad/s.
#
# When the teleop ends (x, Ctrl-C, terminal closed) the launch gets SIGINT, its safety net stops
# both MD400s (VEL 0 + stop + torque off), and this script waits for that. Keep the E-stop in
# hand: the MD400 has no comm watchdog. If a launch is already running, only the teleop starts.
cd "$(dirname "$0")/.."
source setup/env.sh          # before set -u: the ROS setup scripts read unset variables
set -u

record=0; imu=false
for a in "$@"; do
  case "$a" in
    --record) record=1 ;;
    --imu) imu=true ;;
    *) echo "unknown option $a (use --record, --imu)"; exit 2 ;;
  esac
done

if pgrep -f '[r]os2_control_node' > /dev/null; then
  echo "a robot launch is already running — starting the teleop only"
  exec ros2 run oroha_teleop deadman_teleop
fi

stamp=$(date +%Y%m%d_%H%M%S)
dir=data/drive/$stamp; mkdir -p "$dir"
echo "starting the robot launch (log $dir/launch.log) ..."
lpid=$(setup/bg.sh "$dir/launch.log" ros2 launch oroha_bringup robot.launch.py imu:=$imu)
bpid=""

cleanup() {
  trap - EXIT INT TERM
  if [ -n "$bpid" ]; then kill -INT -- "-$bpid" 2>/dev/null; sleep 2; fi
  echo; echo "stopping the launch (safety net stops both MD400s) ..."
  kill -INT -- "-$lpid" 2>/dev/null
  for _ in $(seq 40); do kill -0 "$lpid" 2>/dev/null || break; sleep 0.5; done
  grep -h "oroha safety" "$dir/launch.log" | tail -1
  echo "log: $dir"
}
trap cleanup EXIT INT TERM

ok=0
for _ in $(seq 60); do
  if ! kill -0 "$lpid" 2>/dev/null; then echo "launch exited — see $dir/launch.log"; exit 1; fi
  if ros2 control list_controllers 2>/dev/null | grep -q "diff_cont.*active"; then ok=1; break; fi
  sleep 0.5
done
if [ "$ok" != 1 ]; then echo "diff_cont not active after 30 s — see $dir/launch.log"; exit 1; fi
grep "ERROR" "$dir/launch.log" | head -3

if [ "$record" = 1 ]; then
  bpid=$(setup/bg.sh "$dir/bag.log" ros2 bag record -s mcap -o "$dir/bag" \
    /diff_cont/cmd_vel /diff_cont/cmd_vel_out /diff_cont/odom /joint_states /dynamic_joint_states \
    /oroha_power/sample /oroha_power/calibration_event /imu/data /rosout /diagnostics /tf)
  echo "recording -> $dir/bag"
fi

echo "ready — E-stop in hand. Hold w/s/a/d/q/e to drive, x to quit."
ros2 run oroha_teleop deadman_teleop
