#!/usr/bin/env bash
# Curriculum launcher + optional Path H monitor popup.
set -euo pipefail
WS=/home/jungod/drone_ws
CKPT=$WS/src/drone_rl_planner/checkpoints
LOG=$CKPT/train_sac_curriculum.log
export PYTHONPATH=$WS/src/drone_rl_planner:${PYTHONPATH:-}
cd "$WS"

STAGE="${1:-2}"   # 2, 3, 3b, or 4

# Stop any existing trainer / overnight supervisor
ps -eo pid,cmd | while read -r pid cmd; do
  case "$cmd" in
    *sac_overnight_supervisor.sh*|python3\ -m\ drone_rl_planner.train_sac_polar*)
      kill "$pid" 2>/dev/null || true
      ;;
  esac
done
sleep 2

{
  echo ""
  echo "===== CURRICULUM stage=$STAGE $(date -Is) ====="
} >>"$LOG"

if [[ "$STAGE" == "2" ]]; then
  if [[ ! -f "$CKPT/sac_polar_fast_best.pt" ]]; then
    echo "Missing sac_polar_fast_best.pt — run --fast first" | tee -a "$LOG"
    exit 1
  fi
  nohup python3 -m drone_rl_planner.train_sac_polar --stage2 --device cuda \
    </dev/null >>"$LOG" 2>&1 &
  echo $! >"$CKPT/train_curriculum.pid"
elif [[ "$STAGE" == "3" ]]; then
  if [[ ! -f "$CKPT/sac_polar_mid_best.pt" && ! -f "$CKPT/sac_polar_fast_best.pt" ]]; then
    echo "Missing mid/fast best — run stage2 or --fast first" | tee -a "$LOG"
    exit 1
  fi
  nohup python3 -m drone_rl_planner.train_sac_polar --stage3 --device cuda \
    </dev/null >>"$LOG" 2>&1 &
  echo $! >"$CKPT/train_curriculum.pid"
elif [[ "$STAGE" == "3b" ]]; then
  if [[ ! -f "$CKPT/sac_polar_dense_best.pt" ]]; then
    echo "Missing sac_polar_dense_best.pt — run stage3 first" | tee -a "$LOG"
    exit 1
  fi
  nohup python3 -m drone_rl_planner.train_sac_polar --stage3b --device cuda \
    </dev/null >>"$LOG" 2>&1 &
  echo $! >"$CKPT/train_curriculum.pid"
elif [[ "$STAGE" == "4" ]]; then
  if [[ ! -f "$CKPT/sac_polar_mid_best.pt" && ! -f "$CKPT/sac_polar_fast_best.pt" ]]; then
    echo "Missing mid/fast best — run stage2 or --fast first" | tee -a "$LOG"
    exit 1
  fi
  nohup python3 -m drone_rl_planner.train_sac_polar --stage4 --device cuda \
    </dev/null >>"$LOG" 2>&1 &
  echo $! >"$CKPT/train_curriculum.pid"
else
  echo "Usage: $0 [2|3|3b|4]" >&2
  exit 1
fi
disown || true

# Popup live monitor (same behavior as overnight supervisor).
if [[ -n "${DISPLAY:-}" ]] || [[ -S /tmp/.X11-unix/X0 ]]; then
  export DISPLAY="${DISPLAY:-:0}"
  if ! ps -eo cmd | grep -F 'drone_rl_planner.train_sac_monitor' | grep -qv grep; then
    (
      cd "$WS"
      setsid python3 -m drone_rl_planner.train_sac_monitor \
        </dev/null >>"$CKPT/sac_monitor_gui.log" 2>&1 &
      disown || true
    )
    echo "launched train_sac_monitor on DISPLAY=$DISPLAY"
  fi
fi

sleep 15
ps -eo pid,etime,cmd | awk '/python3 -m drone_rl_planner.train_sac_polar/ && !/awk/'
tail -35 "$LOG"
