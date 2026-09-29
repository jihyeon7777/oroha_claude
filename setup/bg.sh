#!/usr/bin/env bash
# Start a long-running command detached, with SIGINT at its DEFAULT disposition, logging
# to a file, and print its pid:
#   setup/bg.sh <logfile> <command> [args...]
#   pid=$(setup/bg.sh /tmp/launch.log ros2 launch oroha_bringup robot.launch.py)
#   kill -INT -- "-$pid"  # whole process group (ros2 run keeps the node as a child) = Ctrl-C in a terminal: clean shutdown (and the launch safety net)
#
# Why: a non-interactive shell starts background jobs with SIGINT *ignored*, Python keeps an
# ignored SIGINT, and `ros2 launch` then ignores `kill -INT`; SIGTERM makes it exit without
# shutting its children down (seen 2026-09-28, T20260928-02). Resetting SIGINT before exec
# gives the same behaviour as a user's Ctrl-C.
set -euo pipefail
log=$1; shift
setsid python3 -c 'import os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_DFL)
os.execvp(sys.argv[1], sys.argv[1:])' "$@" > "$log" 2>&1 < /dev/null &
echo $!
