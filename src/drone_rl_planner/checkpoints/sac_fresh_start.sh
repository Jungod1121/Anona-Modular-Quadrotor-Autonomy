#!/usr/bin/env bash
# Path H: wipe old SAC weights and train from scratch (large budget, GPU-hot).
set -euo pipefail
WS=/home/jungod/drone_ws
CKPT="$WS/src/drone_rl_planner/checkpoints"
LOG="$CKPT/train_sac_fresh.log"
STEPS="${SAC_FRESH_STEPS:-1000000}"

echo "==== fresh start $(date -Is) steps=$STEPS ===="

# Stop trainers + overnight supervisor
while read -r pid; do
  [[ -z "$pid" ]] && continue
  cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
  if [[ "$cmd" == python3\ -m\ drone_rl_planner.train_sac_polar* ]] || \
     [[ "$cmd" == *sac_overnight_supervisor.sh* ]]; then
    echo "stop pid=$pid"
    kill "$pid" 2>/dev/null || true
  fi
done < <(pgrep -f 'drone_rl_planner.train_sac_polar|sac_overnight_supervisor' || true)
sleep 2
while read -r pid; do
  [[ -z "$pid" ]] && continue
  cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
  if [[ "$cmd" == python3\ -m\ drone_rl_planner.train_sac_polar* ]] || \
     [[ "$cmd" == *sac_overnight_supervisor.sh* ]]; then
    kill -9 "$pid" 2>/dev/null || true
  fi
done < <(pgrep -f 'drone_rl_planner.train_sac_polar|sac_overnight_supervisor' || true)
sleep 1

# Delete Path H checkpoints only (keep Path G sb3_*.zip)
rm -fv "$CKPT"/sac_polar_local.pt "$CKPT"/sac_polar_local_best.pt
# Reset status so UI shows a clean slate
python3 - <<PY
import json
from pathlib import Path
p = Path("$CKPT/sac_training_status.json")
p.write_text(json.dumps({
    "state": "running",
    "algorithm": "DrQ-SAC",
    "steps": 0,
    "total_steps": $STEPS,
    "best_success": 0.0,
    "best_score": 0.0,
    "success_rate": 0.0,
    "collision_rate": 0.0,
    "device": "cuda",
    "fresh": True,
    "target": 0.95,
}, indent=2))
print("status reset", p)
PY

export PYTHONPATH="$WS/src/drone_rl_planner:${PYTHONPATH:-}"
cd "$WS"
: > "$LOG"
echo "starting FRESH train (no --resume) steps=$STEPS" | tee -a "$LOG"
setsid python3 -m drone_rl_planner.train_sac_polar \
  --steps "$STEPS" \
  --eval-every 5000 \
  --eval-episodes 60 \
  --device cuda \
  --dense-heavy \
  --target 0.95 \
  --batch-size 128 \
  --updates-per-step 2 \
  --n-envs 2 \
  </dev/null >>"$LOG" 2>&1 &
disown || true
sleep 8
pgrep -af 'python3 -m drone_rl_planner.train_sac_polar' || echo FAIL_TRAIN
head -15 "$LOG"

# Restart overnight supervisor with larger chunks (will resume ONLY after this run
# produces a new best.pt — which is fine for continuation after 1M).
export SAC_CHUNK=500000
export SAC_MAX_ROUNDS=6
export SAC_TARGET=0.85
bash "$CKPT/launch_overnight.sh"
echo FRESH_DONE
