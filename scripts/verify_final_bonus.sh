#!/usr/bin/env bash
set -eo pipefail
cd /home/jungod/drone_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_planner drone_bringup --symlink-install \
  --allow-overriding drone_planner drone_bringup
source install/setup.bash
bash scripts/cleanup_sim.sh || true
sleep 2

OUT=/tmp/bonus_final
rm -rf "$OUT"
mkdir -p "$OUT"

echo "=== avoidance ==="
ros2 launch drone_bringup avoidance.launch.py use_rviz:=false >"$OUT/avoid_launch.log" 2>&1 &
LPID=$!
sleep 7
ros2 run drone_bringup evaluate_drone --duration 100 --goal-x 17 --goal-y 5 --goal-z 1.5 \
  --output-dir "$OUT/avoid" >"$OUT/avoid_eval.log" 2>&1 || true
kill "$LPID" 2>/dev/null || true
bash scripts/cleanup_sim.sh || true
sleep 2
echo "--- avoid ---"
cat "$OUT/avoid/summary.txt" || true
grep -E 'B-spline|Planned|trajectory_cmd|DynA' "$OUT/avoid_launch.log" | head -15 || true

echo "=== stability wind+imu ==="
ros2 launch drone_bringup stability_demo.launch.py use_rviz:=false run_eval:=false \
  >"$OUT/stab_launch.log" 2>&1 &
LPID=$!
sleep 5
ros2 run drone_bringup evaluate_drone --duration 45 --goal-x 0 --goal-y 0 --goal-z 1.5 \
  --output-dir "$OUT/stab" >"$OUT/stab_eval.log" 2>&1 || true
kill "$LPID" 2>/dev/null || true
bash scripts/cleanup_sim.sh || true
echo "--- stab ---"
cat "$OUT/stab/summary.txt" || true
echo DONE
