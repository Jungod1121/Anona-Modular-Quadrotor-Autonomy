#!/usr/bin/env bash
# Path H supervisor: continuous dense-heavy SAC until best_success >= TARGET.
# Replay buffer is mmap-persisted — restarts continue the same foundation.
set -uo pipefail

WS=/home/jungod/drone_ws
CKPT="$WS/src/drone_rl_planner/checkpoints"
STATUS="$CKPT/sac_training_status.json"
LOG="$CKPT/sac_overnight_supervisor.log"
TRAIN_LOG="$CKPT/train_sac_overnight.log"
TARGET="${SAC_TARGET:-0.90}"
# One long run; early-stops when target hit. Re-attach if process dies.
CHUNK="${SAC_CHUNK:-5000000}"
MAX_ROUNDS="${SAC_MAX_ROUNDS:-20}"
POLL_SEC="${SAC_POLL_SEC:-90}"

mkdir -p "$CKPT"
exec >>"$LOG" 2>&1
echo "==== supervisor start $(date -Is) target=$TARGET chunk=$CHUNK max_rounds=$MAX_ROUNDS persist=mmap ===="

trainer_pids() {
  local pid cmd
  while read -r pid; do
    [[ -z "$pid" ]] && continue
    cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
    if [[ "$cmd" == python3\ -m\ drone_rl_planner.train_sac_polar* ]]; then
      echo "$pid"
    fi
  done < <(pgrep -f 'drone_rl_planner.train_sac_polar' || true)
}

trainer_running() {
  [[ -n "$(trainer_pids)" ]]
}

status_line() {
  python3 - <<'PY'
import json
from pathlib import Path
p = Path("/home/jungod/drone_ws/src/drone_rl_planner/checkpoints/sac_training_status.json")
st = {}
if p.is_file():
    try:
        st = json.loads(p.read_text())
    except Exception:
        pass
best = float(st.get("best_success") or 0.0)
score = float(st.get("best_score") or st.get("score") or -1e9)
steps = int(st.get("steps") or 0)
total = int(st.get("total_steps") or 0)
state = str(st.get("state") or "unknown")
sr = float(st.get("success_rate") or best)
cr = float(st.get("collision_rate") or 0.0)
buf = int(st.get("buf") or 0)
print(f"{state}\t{best:.6f}\t{score:.6f}\t{steps}\t{total}\t{sr:.6f}\t{cr:.6f}\tbuf={buf}")
PY
}

best_from_status() {
  status_line | awk -F'\t' '{print $2}'
}

pick_resume() {
  # Prefer best foundation; last is only a fallback.
  local best="$CKPT/sac_polar_local_best.pt"
  local last="$CKPT/sac_polar_local.pt"
  if [[ -f "$best" ]]; then
    echo "$best"
  elif [[ -f "$last" ]]; then
    echo "$last"
  else
    echo ""
  fi
}

mark_status() {
  local tag="$1"
  python3 - <<PY
import json
from pathlib import Path
p = Path("$STATUS")
st = {}
if p.is_file():
    try:
        st = json.loads(p.read_text())
    except Exception:
        st = {}
st["state"] = "done"
st["supervisor"] = "$tag"
p.write_text(json.dumps(st, indent=2))
PY
}

start_chunk() {
  local resume="$1"
  local steps="$2"
  export PYTHONPATH="$WS/src/drone_rl_planner:${PYTHONPATH:-}"
  cd "$WS"
  echo "$(date -Is) START continuous steps=$steps resume=$resume target=$TARGET lr=1e-4 n_envs=4 persist_buffer"
  {
    echo ""
    echo "===== continuous $(date -Is) steps=$steps target=$TARGET ====="
  } >>"$TRAIN_LOG"
  # Popup monitor if a display is available (Mission Console / desktop).
  if [[ -n "${DISPLAY:-}" ]] || [[ -S /tmp/.X11-unix/X0 ]]; then
    export DISPLAY="${DISPLAY:-:0}"
    if ! ps -eo cmd | grep -F 'drone_rl_planner.train_sac_monitor' | grep -qv grep; then
      (
        export PYTHONPATH="$WS/src/drone_rl_planner:${PYTHONPATH:-}"
        cd "$WS"
        setsid python3 -m drone_rl_planner.train_sac_monitor \
          </dev/null >>"$CKPT/sac_monitor_gui.log" 2>&1 &
        disown || true
      )
      echo "$(date -Is) launched train_sac_monitor on DISPLAY=$DISPLAY"
    fi
  fi
  # Efficient continue: fine-tune LR, more envs to refill/keep buffer warm,
  # mmap replay so crashes/restarts do NOT wipe the foundation.
  setsid python3 -m drone_rl_planner.train_sac_polar \
    --steps "$steps" \
    --eval-every 5000 \
    --eval-episodes 60 \
    --device cuda \
    --dense-heavy \
    --target "$TARGET" \
    --batch-size 128 \
    --updates-per-step 2 \
    --n-envs 4 \
    --buffer-size 250000 \
    --persist-buffer \
    --finetune-lr 1e-4 \
    --resume "$resume" \
    </dev/null >>"$TRAIN_LOG" 2>&1 &
  disown || true
  sleep 12
  if trainer_running; then
    echo "$(date -Is) trainer up pids=$(trainer_pids | tr '\n' ' ')"
    return 0
  fi
  echo "$(date -Is) ERROR trainer failed to start"
  tail -80 "$TRAIN_LOG" || true
  return 1
}

rounds=0
echo "$(date -Is) initial status: $(status_line)"
echo "$(date -Is) trainer_running=$(trainer_running && echo yes || echo no)"

while true; do
  if trainer_running; then
    echo "$(date -Is) RUNNING $(status_line)"
    sleep "$POLL_SEC"
    continue
  fi

  best=$(best_from_status)
  echo "$(date -Is) IDLE best=$best status=$(status_line)"

  awk -v b="$best" -v t="$TARGET" 'BEGIN{exit !(b+1e-9>=t)}' </dev/null
  if [[ $? -eq 0 ]]; then
    echo "$(date -Is) DONE — best_success=$best >= $TARGET"
    mark_status target_reached
    exit 0
  fi

  if (( rounds >= MAX_ROUNDS )); then
    echo "$(date -Is) STOP — max rounds=$MAX_ROUNDS, best=$best"
    mark_status max_rounds
    exit 0
  fi

  resume=$(pick_resume)
  if [[ -z "$resume" ]]; then
    echo "$(date -Is) ERROR no checkpoint to resume"
    exit 1
  fi

  rounds=$((rounds + 1))
  echo "$(date -Is) CONTINUE round=$rounds/$MAX_ROUNDS best=$best < $TARGET (mmap replay kept)"
  if ! start_chunk "$resume" "$CHUNK"; then
    echo "$(date -Is) retry in 60s"
    sleep 60
  fi
done
