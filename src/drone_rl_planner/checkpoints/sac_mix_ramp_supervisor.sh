#!/usr/bin/env bash
# Watch mix curriculum; auto-advance stage5 (30%) → stage6 (50%) when target met.
# Stops / alerts if Live collapses after SGD starts (collision buffer poison).
set -euo pipefail
WS=/home/jungod/drone_ws
CKPT=$WS/src/drone_rl_planner/checkpoints
STATUS=$CKPT/sac_training_status.json
LOG=$CKPT/train_sac_curriculum.log
SUP_LOG=$CKPT/sac_mix_ramp_supervisor.log
export PYTHONPATH=$WS/src/drone_rl_planner:${PYTHONPATH:-}
cd "$WS"

POLL_S="${POLL_S:-90}"
MIN_STEPS_BEFORE_DECLINE="${MIN_STEPS_BEFORE_DECLINE:-20000}"

{
  echo ""
  echo "===== mix ramp supervisor $(date -Is) ====="
} >>"$SUP_LOG"

launch_stage() {
  local stage="$1"
  {
    echo ""
    echo "===== CURRICULUM stage=${stage} ramp $(date -Is) ====="
  } >>"$LOG"
  # Stop any leftover trainer
  pkill -f 'python3 -m drone_rl_planner.train_sac_polar' 2>/dev/null || true
  sleep 2
  nohup python3 -m drone_rl_planner.train_sac_polar --stage"${stage}" --device cuda \
    </dev/null >>"$LOG" 2>&1 &
  echo $! >"$CKPT/train_curriculum.pid"
  echo "$(date -Is) launched --stage${stage} pid=$(cat "$CKPT/train_curriculum.pid")" >>"$SUP_LOG"

  # Ensure monitor is up
  if [[ -S /tmp/.X11-unix/X0 ]]; then
    export DISPLAY="${DISPLAY:-:0}"
    if ! pgrep -f 'drone_rl_planner.train_sac_monitor' >/dev/null 2>&1; then
      setsid python3 -m drone_rl_planner.train_sac_monitor \
        </dev/null >>"$CKPT/sac_monitor_gui.log" 2>&1 &
      disown || true
      echo "$(date -Is) launched monitor" >>"$SUP_LOG"
    fi
  fi
}

read_status() {
  python3 - <<'PY'
import json, sys
from pathlib import Path
p = Path("/home/jungod/drone_ws/src/drone_rl_planner/checkpoints/sac_training_status.json")
if not p.is_file():
    print("missing 0 0 0 0 unknown")
    sys.exit(0)
d = json.loads(p.read_text())
print(
    d.get("state", ""),
    int(d.get("steps") or 0),
    int(d.get("total_steps") or 0),
    float(d.get("best_success") or 0),
    float(d.get("success_rate") if d.get("success_rate") is not None else (d.get("best_success") or 0)),
    float(d.get("target") or 0),
    str(d.get("stage") or ""),
    str(d.get("run_name") or ""),
    float(d.get("mix_dense_p") or -1),
)
PY
}

# If nothing running and mixb done at 85%, start stage5.
if ! pgrep -f 'python3 -m drone_rl_planner.train_sac_polar' >/dev/null 2>&1; then
  launch_stage 5
fi

prev_live=-1
decline_hits=0

while true; do
  sleep "$POLL_S"
  if [[ ! -f "$STATUS" ]]; then
    continue
  fi
  # shellcheck disable=SC2046
  set -- $(read_status)
  state=$1; steps=$2; total=$3; best=$4; live=$5; target=$6; stage=$7; run=$8; pdense=$9

  running=0
  if pgrep -f 'python3 -m drone_rl_planner.train_sac_polar' >/dev/null 2>&1; then
    running=1
  fi

  echo "$(date -Is) run=$running state=$state stage=$stage run_name=$run steps=$steps/$total best=$best live=$live target=$target p_dense=$pdense" >>"$SUP_LOG"

  # Decline guard: after warm-up, live << best and dropping → kill & stop ramp.
  if [[ "$running" -eq 1 && "$steps" -ge "$MIN_STEPS_BEFORE_DECLINE" ]]; then
    set +e
    python3 - <<PY >>"$SUP_LOG" 2>&1
best=float("$best"); live=float("$live"); prev=float("$prev_live")
bad = (best >= 0.40 and live + 0.25 < best) or (prev >= 0 and live + 0.10 < prev and live < 0.35)
print(f"decline_check live={live:.3f} best={best:.3f} prev={prev:.3f} bad={bad}")
raise SystemExit(1 if bad else 0)
PY
    rc=$?
    set -e
    if [[ $rc -eq 1 ]]; then
      decline_hits=$((decline_hits + 1))
    else
      decline_hits=0
    fi
    if (( decline_hits >= 3 )); then
      echo "$(date -Is) DECLINE DETECTED — stopping trainer, supervisor exit" >>"$SUP_LOG"
      pkill -f 'python3 -m drone_rl_planner.train_sac_polar' 2>/dev/null || true
      exit 2
    fi
  fi
  prev_live=$live

  if [[ "$running" -eq 1 ]]; then
    continue
  fi

  # Trainer idle
  if [[ "$state" != "done" ]]; then
    echo "$(date -Is) idle unexpected state=$state — wait" >>"$SUP_LOG"
    continue
  fi

  # Advance ladder
  met=$(python3 -c "print(1 if float('$best')+1e-9 >= float('$target' or 0) and float('$target' or 0)>0 else 0)")
  if [[ "$stage" == "5" && "$met" == "1" ]]; then
    echo "$(date -Is) stage5 target met ($best>=$target) → stage6" >>"$SUP_LOG"
    launch_stage 6
    prev_live=-1
    decline_hits=0
    continue
  fi
  if [[ "$stage" == "6" && "$met" == "1" ]]; then
    echo "$(date -Is) stage6 target met ($best>=$target) — RAMP COMPLETE, exit 0" >>"$SUP_LOG"
    exit 0
  fi

  # Done but missed target: do not auto-continue (avoids collision-buffer poison).
  echo "$(date -Is) stage=$stage done but best=$best < target=$target — stop (no blind continue)" >>"$SUP_LOG"
  exit 1
done
