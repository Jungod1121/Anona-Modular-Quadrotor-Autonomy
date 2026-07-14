#!/usr/bin/env bash
set -eo pipefail
cd /home/jungod/drone_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_planner drone_bringup --symlink-install \
  --allow-overriding drone_planner drone_bringup
source install/setup.bash
bash scripts/cleanup_sim.sh || true
OUT=/tmp/sf2
rm -rf "$OUT"
mkdir -p "$OUT"
timeout 90 ros2 launch drone_bringup shared_field.launch.py use_rviz:=false \
  >"$OUT/l.log" 2>&1 || true
bash scripts/cleanup_sim.sh || true
echo '=== keys ==='
grep -E 'Planning|Running A|A\* finished|Planned|failed|New goal' "$OUT/l.log" | head -50 || true
echo DONE
