#!/usr/bin/env bash
# Restart Path H trainer with GPU-hungry settings; keep overnight supervisor.
set -euo pipefail
WS=/home/jungod/drone_ws
CKPT="$WS/src/drone_rl_planner/checkpoints"
LOG="$CKPT/train_sac_gpu_hot.log"
RESUME="$CKPT/sac_polar_local_best.pt"
LAST="$CKPT/sac_polar_local.pt"
[[ -f "$LAST" && "$LAST" -nt "$RESUME" ]] && RESUME="$LAST"

# Stop only the python trainer
while read -r pid; do
  [[ -z "$pid" ]] && continue
  cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
  if [[ "$cmd" == python3\ -m\ drone_rl_planner.train_sac_polar* ]]; then
    echo "stop trainer pid=$pid"
    kill "$pid" 2>/dev/null || true
  fi
done < <(pgrep -f 'drone_rl_planner.train_sac_polar' || true)
sleep 2
while read -r pid; do
  [[ -z "$pid" ]] && continue
  cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
  if [[ "$cmd" == python3\ -m\ drone_rl_planner.train_sac_polar* ]]; then
    kill -9 "$pid" 2>/dev/null || true
  fi
done < <(pgrep -f 'drone_rl_planner.train_sac_polar' || true)

export PYTHONPATH="$WS/src/drone_rl_planner:${PYTHONPATH:-}"
cd "$WS"
echo "resume=$RESUME" | tee "$LOG"
setsid python3 -m drone_rl_planner.train_sac_polar \
  --steps 500000 \
  --eval-every 5000 \
  --eval-episodes 60 \
  --device cuda \
  --dense-heavy \
  --target 0.85 \
  --batch-size 256 \
  --updates-per-step 12 \
  --n-envs 4 \
  --resume "$RESUME" \
  </dev/null >>"$LOG" 2>&1 &
disown || true
sleep 8
pgrep -af 'python3 -m drone_rl_planner.train_sac_polar' || echo FAIL
head -20 "$LOG"

# Ensure overnight supervisor is still watching
if ! pgrep -f sac_overnight_supervisor.sh >/dev/null; then
  bash "$CKPT/launch_overnight.sh"
fi
echo DONE
