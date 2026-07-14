#!/usr/bin/env bash
set -eo pipefail
cd /home/jungod/drone_ws
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
bash scripts/cleanup_sim.sh || true
OUT=/tmp/bonus_verify
rm -rf "$OUT"
mkdir -p "$OUT"

echo "=== stability (wind+IMU) ==="
ros2 launch drone_bringup stability_demo.launch.py use_rviz:=false run_eval:=false \
  >"$OUT/stab_launch.log" 2>&1 &
LPID=$!
sleep 5
ros2 run drone_bringup evaluate_drone --duration 50 --goal-x 0 --goal-y 0 --goal-z 1.5 \
  --output-dir "$OUT/stab" >"$OUT/stab_eval.log" 2>&1 || true
kill "$LPID" 2>/dev/null || true
bash scripts/cleanup_sim.sh || true
echo "--- stab summary ---"
cat "$OUT/stab/summary.txt" 2>/dev/null || { echo "NO STAB SUMMARY"; cat "$OUT/stab_eval.log"; }
tail -5 "$OUT/stab_launch.log" || true

sleep 2
echo "=== avoidance (B-spline on) ==="
ros2 launch drone_bringup avoidance.launch.py use_rviz:=false \
  >"$OUT/avoid_launch.log" 2>&1 &
LPID=$!
sleep 6
ros2 run drone_bringup evaluate_drone --duration 90 --goal-x 17 --goal-y 5 --goal-z 1.5 \
  --output-dir "$OUT/avoid" >"$OUT/avoid_eval.log" 2>&1 || true
kill "$LPID" 2>/dev/null || true
bash scripts/cleanup_sim.sh || true
echo "--- avoid summary ---"
cat "$OUT/avoid/summary.txt" 2>/dev/null || { echo "NO AVOID SUMMARY"; cat "$OUT/avoid_eval.log"; }
grep -E 'B-spline|DynA|Planned path|EMERGENCY|success|ready' "$OUT/avoid_launch.log" | head -30 || true

echo "=== multi_drone smoke (12s) ==="
timeout 12 ros2 launch drone_bringup multi_drone.launch.py use_rviz:=false \
  >"$OUT/multi.log" 2>&1 || true
bash scripts/cleanup_sim.sh || true
grep -E 'ready|Error|Traceback|uav0|uav1' "$OUT/multi.log" | head -40 || true
echo DONE
