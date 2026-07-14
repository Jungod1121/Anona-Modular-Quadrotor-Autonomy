#!/usr/bin/env bash
# Kill leftover drone_ws simulation / evaluation processes before a fresh launch.
set -euo pipefail

pkill -9 -f 'ros2 launch drone_bringup' 2>/dev/null || true
pkill -9 -f 'install/drone_bringup/lib/drone_bringup' 2>/dev/null || true
pkill -9 -f 'install/drone_(dynamics|controller|map|planner|visualization)/lib' 2>/dev/null || true
pkill -9 -f 'send_goal|evaluate_drone|waypoint_publisher' 2>/dev/null || true
sleep 0.5

left=$(pgrep -af 'install/drone_|drone_bringup/(send_goal|evaluate|waypoint)' 2>/dev/null | wc -l || echo 0)
echo "cleanup_sim: ${left} leftover process(es)"
