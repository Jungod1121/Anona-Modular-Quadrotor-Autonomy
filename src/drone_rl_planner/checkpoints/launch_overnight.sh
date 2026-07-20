#!/usr/bin/env bash
# Launch overnight supervisor detached (safe from pgrep self-kill).
LOG=/home/jungod/drone_ws/src/drone_rl_planner/checkpoints/sac_overnight_supervisor.log
SCRIPT=/home/jungod/drone_ws/src/drone_rl_planner/checkpoints/sac_overnight_supervisor.sh
chmod +x "$SCRIPT"
# Kill previous supervisor only
while read -r pid; do
  [[ -z "$pid" ]] && continue
  cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
  if [[ "$cmd" == *sac_overnight_supervisor.sh* ]]; then
    kill "$pid" 2>/dev/null || true
  fi
done < <(pgrep -f sac_overnight_supervisor || true)
sleep 1
setsid bash "$SCRIPT" </dev/null >/dev/null 2>&1 &
disown || true
sleep 2
pgrep -af sac_overnight_supervisor || echo SUPERVISOR_MISSING
tail -5 "$LOG" 2>/dev/null || true
