#!/usr/bin/env bash
# Watch Path H dense training; if budget ends below target, continue with
# more steps while keeping the mmap replay buffer.
set -euo pipefail
WS=/home/jungod/drone_ws
CKPT=$WS/src/drone_rl_planner/checkpoints
STATUS=$CKPT/sac_training_status.json
LOG=$CKPT/train_sac_curriculum.log
SUP_LOG=$CKPT/sac_dense_continue_supervisor.log
export PYTHONPATH=$WS/src/drone_rl_planner:${PYTHONPATH:-}
cd "$WS"

TARGET="${TARGET:-0.80}"
EXTRA_STEPS="${EXTRA_STEPS:-600000}"
POLL_S="${POLL_S:-60}"
MAX_CONTINUES="${MAX_CONTINUES:-3}"

cont=0
{
  echo ""
  echo "===== dense continue supervisor $(date -Is) target=$TARGET extra=$EXTRA_STEPS ====="
} >>"$SUP_LOG"

while true; do
  if [[ ! -f "$STATUS" ]]; then
    sleep "$POLL_S"
    continue
  fi
  state=$(python3 -c "import json;print(json.load(open('$STATUS')).get('state',''))" 2>/dev/null || echo '')
  best=$(python3 -c "import json;print(float(json.load(open('$STATUS')).get('best_success') or 0))" 2>/dev/null || echo 0)
  steps=$(python3 -c "import json;print(int(json.load(open('$STATUS')).get('steps') or 0))" 2>/dev/null || echo 0)
  total=$(python3 -c "import json;print(int(json.load(open('$STATUS')).get('total_steps') or 0))" 2>/dev/null || echo 0)

  # Still training?
  if pgrep -f 'python3 -m drone_rl_planner.train_sac_polar' >/dev/null 2>&1; then
    echo "$(date -Is) running steps=$steps/$total best=$best" >>"$SUP_LOG"
    sleep "$POLL_S"
    continue
  fi

  # Trainer idle
  if [[ "$state" != "done" ]]; then
    echo "$(date -Is) idle state=$state — waiting" >>"$SUP_LOG"
    sleep "$POLL_S"
    continue
  fi

  set +e
  python3 - <<PY >>"$SUP_LOG" 2>&1
best=float("$best"); target=float("$TARGET")
print(f"trainer done best={best:.1%} target={target:.0%}")
if best + 1e-9 >= target:
    raise SystemExit(0)
raise SystemExit(1)
PY
  rc=$?
  set -e
  if [[ $rc -eq 0 ]]; then
    echo "$(date -Is) TARGET MET best=$best — supervisor exit" >>"$SUP_LOG"
    exit 0
  fi

  if (( cont >= MAX_CONTINUES )); then
    echo "$(date -Is) max continues ($MAX_CONTINUES) reached best=$best — stop" >>"$SUP_LOG"
    exit 1
  fi
  cont=$((cont + 1))

  RESUME=$CKPT/sac_polar_dense_best.pt
  [[ -f "$RESUME" ]] || RESUME=$CKPT/sac_polar_dense.pt
  if [[ ! -f "$RESUME" ]]; then
    echo "$(date -Is) no dense checkpoint to resume" >>"$SUP_LOG"
    exit 1
  fi

  echo "$(date -Is) CONTINUE #$cont resume=$RESUME steps=$EXTRA_STEPS (keep buffer)" >>"$SUP_LOG"
  {
    echo ""
    echo "===== CURRICULUM dense-continue #$cont $(date -Is) ====="
  } >>"$LOG"

  # Keep buffer: do NOT pass --reset-buffer
  nohup python3 -m drone_rl_planner.train_sac_polar \
    --dense-heavy --device cuda \
    --name sac_polar_dense \
    --resume "$RESUME" \
    --finetune-lr 5e-5 \
    --steps "$EXTRA_STEPS" \
    --target "$TARGET" \
    --eval-episodes 60 \
    --persist-buffer \
    </dev/null >>"$LOG" 2>&1 &
  echo $! >"$CKPT/train_curriculum.pid"
  echo "$(date -Is) started pid=$(cat $CKPT/train_curriculum.pid)" >>"$SUP_LOG"
  sleep 30
done
