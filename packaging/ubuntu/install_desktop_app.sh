#!/usr/bin/env bash
# Install Drone WS Console into the Ubuntu app menu (user-local, no root).
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS="$(cd "$HERE/../.." && pwd)"
export DRONE_WS="${DRONE_WS:-$WS}"

BIN_DIR="${XDG_BIN_HOME:-$HOME/.local/bin}"
APP_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
ICON_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/icons/hicolor/scalable/apps"

mkdir -p "$BIN_DIR" "$APP_DIR" "$ICON_DIR"
chmod +x "$HERE/drone-ws-dashboard"

# Thin wrapper so the menu entry always points at this workspace
cat > "$BIN_DIR/drone-ws-dashboard" <<EOF
#!/usr/bin/env bash
export DRONE_WS="$DRONE_WS"
exec "$HERE/drone-ws-dashboard" "\$@"
EOF
chmod 755 "$BIN_DIR/drone-ws-dashboard"

install -m 644 "$HERE/drone-ws-dashboard.svg" "$ICON_DIR/drone-ws-dashboard.svg"

sed -e "s|@BIN@|$BIN_DIR/drone-ws-dashboard|g" \
    -e "s|@ICON@|drone-ws-dashboard|g" \
    "$HERE/drone-ws-dashboard.desktop.in" > "$APP_DIR/drone-ws-dashboard.desktop"
chmod 644 "$APP_DIR/drone-ws-dashboard.desktop"

update-desktop-database "$APP_DIR" 2>/dev/null || true
gtk-update-icon-cache -f -t "${XDG_DATA_HOME:-$HOME/.local/share}/icons/hicolor" 2>/dev/null || true

echo "Installed."
echo "  Command:   drone-ws-dashboard"
echo "  Menu name: Drone WS Console / 无人机仿真控制台"
echo "  Workspace: $DRONE_WS"
echo
echo "Open Activities → search「无人机」or「Drone WS」."
if ! command -v drone-ws-dashboard >/dev/null 2>&1; then
  echo
  echo "Note: add ~/.local/bin to PATH if the command is not found:"
  echo "  echo 'export PATH=\"\$HOME/.local/bin:\$PATH\"' >> ~/.bashrc"
fi
if [[ ! -f "$DRONE_WS/install/setup.bash" ]]; then
  echo
  echo "WARNING: missing $DRONE_WS/install/setup.bash — build first:"
  echo "  cd \"$DRONE_WS\" && colcon build --symlink-install"
fi
