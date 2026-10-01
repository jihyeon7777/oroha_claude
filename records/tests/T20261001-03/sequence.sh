#!/bin/bash
# T20261001-03 sequence (lifted). Every motion command ends by itself (-t).
cd /home/oroha/oroha_claude && source setup/env.sh
S=/tmp/claude-1000/-home-oroha-oroha-claude/2785c482-c01f-40e1-806f-0b49dfda47d3/scratchpad
L=$S/b2_phases.log; : > $L
ph(){ echo "$1 $(date +%s.%N)" >> $L; }
pp=$(setup/bg.sh $S/b2_pw.log ros2 run oroha_power power_node); sleep 4
bp=$(setup/bg.sh $S/b2_bag.log ros2 bag record -o data/tests/T20261001-03/bag /oroha_power/sample /oroha_power/calibration_event /joint_states /dynamic_joint_states /diff_cont/cmd_vel /diff_cont/cmd_vel_out /rosout /diagnostics); sleep 4
ph torque_off_rest_start; sleep 12; ph torque_off_rest_end
hp=$(setup/bg.sh $S/b2_hw.log ros2 launch oroha_bringup robot.launch.py power:=false); sleep 14
ros2 control list_controllers >> $L
ph enabled_rest_start; sleep 12; ph enabled_rest_end
ros2 service call /oroha_power/zero std_srvs/srv/Trigger >> $L; sleep 3
for v in 0.2 0.4 0.6 0.76 -0.4; do
  ph step_v${v}_start
  ros2 topic pub -r 10 -t 70 /diff_cont/cmd_vel geometry_msgs/msg/TwistStamped "{header: auto, twist: {linear: {x: $v}}}" > /dev/null
  ph step_v${v}_end; sleep 6
done
for w in 2.0 -2.0; do
  ph step_w${w}_start
  ros2 topic pub -r 10 -t 70 /diff_cont/cmd_vel geometry_msgs/msg/TwistStamped "{header: auto, twist: {angular: {z: $w}}}" > /dev/null
  ph step_w${w}_end; sleep 6
done
ph final_rest_end
kill -INT -- -$bp; sleep 3
kill -INT -- -$hp; sleep 10
kill -INT -- -$pp; sleep 3
grep -E "oroha safety" $S/b2_hw.log >> $L
echo DONE >> $L
