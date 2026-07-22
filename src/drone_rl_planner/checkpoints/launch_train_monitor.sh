#!/usr/bin/env bash
# Popup live Path H training monitor (Tk GUI).
set -euo pipefail
WS="${WS:-/home/jungod/drone_ws}"
export PYTHONPATH="$WS/src/drone_rl_planner:${PYTHONPATH:-}"
# Prefer local display
export DISPLAY="${DISPLAY:-:1}"
cd "$WS"
exec python3 -m drone_rl_planner.train_sac_monitor "$@"
