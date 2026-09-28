#!/usr/bin/env bash
# One-time system setup that needs root. Run from the workspace root:
#   sudo bash setup/install_system.sh
# Installs the ros2_control stack + tools, the udev rules for stable device
# names (/dev/oroha_md400, /dev/oroha_pico, /dev/oroha_um7), sets the timezone
# to Asia/Seoul, turns chrony on (Pi = NTP server for the GT laptop) and sets the
# FTDI latency timer to 1 ms. Safe to re-run.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)

apt-get update
apt-get install -y \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro \
  ros-jazzy-diagnostic-updater ros-jazzy-rqt-robot-monitor ros-jazzy-rviz-imu-plugin \
  chrony python3-pandas

install -m 644 "$here/udev/99-oroha.rules" /etc/udev/rules.d/99-oroha.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
# latency_timer rule is ACTION=="add": replay add events, and set it directly for the running adapter
udevadm trigger --action=add --subsystem-match=usb-serial
for f in /sys/bus/usb-serial/drivers/ftdi_sio/*/latency_timer; do [ -e "$f" ] && echo 1 > "$f"; done

timedatectl set-timezone Asia/Seoul

install -d /etc/chrony/conf.d
install -m 644 "$here/chrony/oroha.conf" /etc/chrony/conf.d/oroha.conf
systemctl restart chrony

echo "--- devices ---"; ls -l /dev/oroha_* 2>/dev/null || echo "(no /dev/oroha_* yet — replug devices)"
echo "--- FTDI latency_timer (want 1) ---"; cat /sys/bus/usb-serial/drivers/ftdi_sio/*/latency_timer 2>/dev/null || echo "(no FTDI adapter)"
echo "--- time ---";    timedatectl | sed -n '1,4p'
echo "--- chrony ---";  chronyc tracking | sed -n '1,4p'
