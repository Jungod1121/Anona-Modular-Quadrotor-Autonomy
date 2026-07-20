#!/usr/bin/env bash
# Kill leftover drone_ws simulation / evaluation processes before a fresh launch.
# Prefer process-group ownership when the dashboard / acceptance runner started the tree;
# this script is the safety net for orphans.
set -euo pipefail

patterns=(
  'ros2 launch drone_bringup'
  'planner_sim.launch.py'
  'fuel_explore.launch.py'
  'ego_avoidance.launch.py'
  'gcopter_avoidance.launch.py'
  'mighty_avoidance.launch.py'
  'fast_planner_avoidance.launch.py'
  'rl_avoidance.launch.py'
  'sac_avoidance.launch.py'
  'ego_swarm.launch.py'
  'shared_field.launch.py'
  'formation.launch.py'
  # Never match lib/drone_bringup/dashboard — acceptance runs from the web UI.
  'lib/drone_bringup/cloud_bridge'
  'lib/drone_bringup/ego_cmd_bridge'
  'lib/drone_bringup/evaluate_drone'
  'lib/drone_bringup/formation_coordinator'
  'lib/drone_bringup/map_adapter'
  'lib/drone_bringup/mighty_cmd_bridge'
  'lib/drone_bringup/pose_to_path_goal'
  'lib/drone_bringup/send_goal'
  'lib/drone_bringup/waypoint_publisher'
  'lib/drone_dynamics/'
  'lib/drone_controller/'
  'lib/drone_map/'
  'lib/drone_planner/'
  'lib/drone_visualization/'
  'lib/drone_exploration/'
  'lib/drone_rl_planner/'
  'lib/ego_planner/'
  'lib/gcopter/'
  'lib/mockamap/'
  'lib/map_generator/'
  'lib/traj_utils/'
  'lib/mighty/'
  'lib/plan_manage/'
  'dynamics_node'
  'controller_node'
  'planner_node'
  'map_node'
  'viz_node'
  'vfh_planner_node'
  'rl_planner_node'
  'sac_planner_node'
  'safety_supervisor_node'
  'ego_cmd_bridge'
  'mighty_cmd_bridge'
  'cloud_bridge'
  'evaluate_drone'
  'send_goal'
  'waypoint_publisher'
  'formation_coordinator'
  'exploration_fsm'
  'rviz2'
)

for pat in "${patterns[@]}"; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 0.8

left=$(pgrep -af 'lib/drone_|ros2 launch drone_bringup|vfh_planner|sac_planner|safety_supervisor' 2>/dev/null \
  | grep -v 'lib/drone_bringup/dashboard' \
  | grep -v 'ros2 run drone_bringup dashboard' \
  | wc -l || true)
echo "cleanup_sim: ${left} leftover process(es)"
