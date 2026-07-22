#!/usr/bin/env bash
# Short-term high-success Path H training (shared DrQ + easy scenes).
set -euo pipefail
WS=/home/jungod/drone_ws
CKPT=$WS/src/drone_rl_planner/checkpoints
LOG=$CKPT/train_sac_fast.log
export PYTHONPATH=$WS/src/drone_rl_planner:${PYTHONPATH:-}
cd "$WS"

# Stop dense overnight + any prior trainer
ps -eo pid,cmd | while read -r pid cmd; do
  case "$cmd" in
    *sac_overnight_supervisor.sh*|python3\ -m\ drone_rl_planner.train_sac_polar*)
      kill "$pid" 2>/dev/null || true
      ;;
  esac
done
sleep 2
ps -eo pid,cmd | while read -r pid cmd; do
  case "$cmd" in
    python3\ -m\ drone_rl_planner.train_sac_polar*)
      kill -KILL "$pid" 2>/dev/null || true
      ;;
  esac
done

{
  echo ""
  echo "===== FAST start $(date -Is) ====="
} >>"$LOG"

setsid python3 -m drone_rl_planner.train_sac_polar --fast --device cuda \
  </dev/null >>"$LOG" 2>&1 &
disown || true
sleep 8
ps -eo pid,etime,cmd | awk '/python3 -m drone_rl_planner.train_sac_polar/ && !/awk/'
tail -20 "$LOG"
