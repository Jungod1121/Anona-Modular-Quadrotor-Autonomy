#!/usr/bin/env bash
set -eo pipefail
cd /home/jungod/drone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/cleanup_sim.sh || true
OUT=/tmp/shared_field_test
rm -rf "$OUT"
mkdir -p "$OUT"

ros2 launch drone_bringup shared_field.launch.py use_rviz:=false >"$OUT/launch.log" 2>&1 &
LPID=$!
sleep 70
kill "$LPID" 2>/dev/null || true
bash scripts/cleanup_sim.sh || true

echo '=== peer / plan lines ==='
grep -E 'Peer avoidance|Planned path|New goal|Map ingested|B-spline|EMERGENCY|ready' "$OUT/launch.log" | head -40 || true
echo '=== DONE ==='
