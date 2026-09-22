#!/usr/bin/env bash
# Fetch the pinned external sources, apply local patches (if any) and resolve
# ROS dependencies. Idempotent; run from anywhere:
#   bash setup/bootstrap.sh
# System packages are handled separately by setup/install_system.sh (sudo).
set -euo pipefail
ws=$(cd "$(dirname "$0")/.." && pwd)
cd "$ws"

mkdir -p src/external
vcs import --input oroha.repos src/external
echo "--- exact versions (record these in records/changes.md) ---"
vcs export --exact src/external

bash patches/apply_patches.sh

source /opt/ros/jazzy/setup.bash
if ! rosdep update >/dev/null 2>&1; then
  echo "rosdep update failed — run 'sudo rosdep init && rosdep update' once" >&2
fi
rosdep install --from-paths src --ignore-src -r -y
