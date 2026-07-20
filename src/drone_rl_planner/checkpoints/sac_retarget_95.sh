#!/usr/bin/env bash
# Retarget running Path H train to 95% + larger overnight chunks (keep weights if any).
set -euo pipefail
WS=/home/jungod/drone_ws
CKPT="$WS/src/drone_rl_planner/checkpoints"
LOG="$CKPT/train_sac_fresh.log"
STEPS=1000000

# Stop trainer + supervisor
while read -r pid; do
  [[ -z "$pid" ]] && continue
  cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
  if [[ "$cmd" == python3\ -m\ drone_rl_planner.train_sac_polar* ]] || \
     [[ "$cmd" == *sac_overnight_supervisor.sh* ]]; then
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

RESUME_ARGS=()
if [[ -f "$CKPT/sac_polar_local_best.pt" ]]; then
  RESUME_ARGS=(--resume "$CKPT/sac_polar_local_best.pt")
elif [[ -f "$CKPT/sac_polar_local.pt" ]]; then
  RESUME_ARGS=(--resume "$CKPT/sac_polar_local.pt")
fi

export PYTHONPATH="$WS/src/drone_rl_planner:${PYTHONPATH:-}"
cd "$WS"
echo "==== retarget 95% $(date -Is) resume=${RESUME_ARGS[*]:-none} ====" | tee -a "$LOG"
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
  "${RESUME_ARGS[@]}" \
  </dev/null >>"$LOG" 2>&1 &
disown || true
sleep 6
pgrep -af 'python3 -m drone_rl_planner.train_sac_polar' | grep -v sandbox | head -2

export SAC_TARGET=0.95
export SAC_CHUNK=800000
export SAC_MAX_ROUNDS=8
bash "$CKPT/launch_overnight.sh"
echo OK
