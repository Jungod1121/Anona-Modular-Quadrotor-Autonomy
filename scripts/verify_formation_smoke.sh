#!/usr/bin/env bash
set -eo pipefail
cd /home/jungod/drone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/cleanup_sim.sh || true
OUT=/tmp/formation_smoke
rm -rf "$OUT"
mkdir -p "$OUT"
timeout 25 ros2 launch drone_bringup formation.launch.py use_rviz:=false formation:=v \
  >"$OUT/l.log" 2>&1 || true
bash scripts/cleanup_sim.sh || true
echo '=== formation smoke ==='
grep -E 'Formation|ready|New goal|Planned|Peer|Error|Traceback' "$OUT/l.log" | head -40 || true
echo DONE
