#!/usr/bin/env bash
# One-time system setup that needs root. Run from the workspace root:
#   sudo bash setup/install_system.sh
# Installs the ros2_control stack + tools, the udev rules for stable device
# names (/dev/oroha_md400, /dev/oroha_pico, /dev/oroha_um7), sets the timezone
# to Asia/Seoul and turns chrony on (Pi = NTP server for the GT laptop).
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)

apt-get update
apt-get install -y \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro \
  ros-jazzy-diagnostic-updater ros-jazzy-rqt-robot-monitor \
  chrony python3-pandas

install -m 644 "$here/udev/99-oroha.rules" /etc/udev/rules.d/99-oroha.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty

timedatectl set-timezone Asia/Seoul

install -d /etc/chrony/conf.d
install -m 644 "$here/chrony/oroha.conf" /etc/chrony/conf.d/oroha.conf
systemctl restart chrony

echo "--- devices ---"; ls -l /dev/oroha_* 2>/dev/null || echo "(no /dev/oroha_* yet — replug devices)"
echo "--- time ---";    timedatectl | sed -n '1,4p'
echo "--- chrony ---";  chronyc tracking | sed -n '1,4p'
