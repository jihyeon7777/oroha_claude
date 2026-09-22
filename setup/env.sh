# source setup/env.sh   (from any directory)
source /opt/ros/jazzy/setup.bash
_oroha_ws=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [ -f "$_oroha_ws/install/setup.bash" ]; then
  source "$_oroha_ws/install/setup.bash"
fi
export OROHA_WS="$_oroha_ws"
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
# If the GT laptop cannot be discovered over WiFi (multicast blocked), list it here:
# export ROS_STATIC_PEERS=192.168.5.xxx
export PATH="$HOME/.local/bin:$PATH"   # mpremote
# ament_python console scripts live in install/<pkg>/lib/<pkg>; expose them as plain commands
for _p in oroha_tools oroha_experiment oroha_power; do
  [ -d "$OROHA_WS/install/$_p/lib/$_p" ] && export PATH="$OROHA_WS/install/$_p/lib/$_p:$PATH"
done
unset _p
unset _oroha_ws
