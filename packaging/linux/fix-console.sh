#!/usr/bin/env bash
# One-shot: clear stuck dashboards, sync console code, relaunch desktop app.
set -eo pipefail

WS="${DRONE_WS:-$HOME/drone_ws}"
cd "$WS"

echo "==> Stopping stuck dashboard backends (needs sudo if started as root)…"
sudo fuser -k 8765/tcp 2>/dev/null || true
sudo pkill -9 -f 'lib/drone_bringup/dashboard' 2>/dev/null || true
pkill -9 -f 'lib/drone_bringup/dashboard' 2>/dev/null || true
sleep 1

echo "==> Building drone_bringup…"
# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install --packages-select drone_bringup

echo "==> Syncing install tree (map projection + desktop fixes)…"
INST="$WS/install/drone_bringup/lib/python3.10/site-packages/drone_bringup"
SRC="$WS/src/drone_bringup/drone_bringup"
cp -a "$SRC/dashboard_server.py" "$SRC/desktop_app.py" "$SRC/map_adapter_node.py" "$INST/"
rm -rf "$INST/dashboard_static"
cp -a "$SRC/dashboard_static" "$INST/dashboard_static"
find "$INST/dashboard_static" -name '__init__.py' -delete 2>/dev/null || true

echo "==> Reinstalling desktop launcher…"
bash "$WS/packaging/linux/install.sh"

echo "==> Starting Drone WS Console…"
# shellcheck disable=SC1091
set +u
source "$WS/install/setup.bash"
set -u
exec drone-ws-console
