#!/usr/bin/env bash
set -eo pipefail
cd /home/jungod/drone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select drone_planner --symlink-install --allow-overriding drone_planner
source install/setup.bash
bash scripts/cleanup_sim.sh || true
OUT=/tmp/bonus_verify2
rm -rf "$OUT"
mkdir -p "$OUT"
ros2 launch drone_bringup avoidance.launch.py use_rviz:=false >"$OUT/launch.log" 2>&1 &
LPID=$!
sleep 7
ros2 run drone_bringup evaluate_drone --duration 100 --goal-x 17 --goal-y 5 --goal-z 1.5 \
  --output-dir "$OUT/avoid" >"$OUT/eval.log" 2>&1 || true
kill "$LPID" 2>/dev/null || true
bash scripts/cleanup_sim.sh || true
echo '--- summary ---'
cat "$OUT/avoid/summary.txt" || true
echo '--- planner ---'
grep -E 'B-spline|Planned|DynA|ready' "$OUT/launch.log" | head -20 || true
