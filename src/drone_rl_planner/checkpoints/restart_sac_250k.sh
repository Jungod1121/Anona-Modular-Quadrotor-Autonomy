#!/usr/bin/env bash
set -euo pipefail
WS=/home/jungod/drone_ws
CKPT="$WS/src/drone_rl_planner/checkpoints"
LOG="$CKPT/train_sac_dense_v3.log"
RESUME="$CKPT/sac_polar_local_best.pt"
LAST="$CKPT/sac_polar_local.pt"
if [[ -f "$LAST" && "$LAST" -nt "$RESUME" ]]; then
  RESUME="$LAST"
fi

# Kill existing trainer only (match python cmdline, skip bash wrappers).
while read -r pid; do
  [[ -z "$pid" ]] && continue
  cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
  if [[ "$cmd" == python3\ -m\ drone_rl_planner.train_sac_polar* ]]; then
    echo "stopping pid=$pid"
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
sleep 1

export PYTHONPATH="$WS/src/drone_rl_planner:${PYTHONPATH:-}"
cd "$WS"
echo "resume=$RESUME" | tee "$LOG"
setsid python3 -m drone_rl_planner.train_sac_polar \
  --steps 250000 --eval-every 5000 --eval-episodes 60 --device cuda --dense-heavy \
  --resume "$RESUME" \
  </dev/null >>"$LOG" 2>&1 &
echo "started pid=$!"
sleep 5
pgrep -a -f 'python3 -m drone_rl_planner.train_sac_polar' || echo 'FAILED to start'
head -20 "$LOG"
