#!/usr/bin/env bash
# Remove user-local Drone WS Console installation (does not delete drone_ws).
set -euo pipefail

BIN_DIR="${XDG_BIN_HOME:-$HOME/.local/bin}"
APP_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
ICON_BASE="${XDG_DATA_HOME:-$HOME/.local/share}/icons/hicolor"
CFG_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/drone-ws"
SHARE_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/drone-ws-console"

rm -f "$BIN_DIR/drone-ws-console" "$BIN_DIR/drone-ws-update" "$BIN_DIR/drone-ws-console-update"
rm -f "$APP_DIR/drone-ws-console.desktop"
rm -f "$ICON_BASE/scalable/apps/drone-ws-console.svg"
rm -f "$ICON_BASE/48x48/apps/drone-ws-console.png"
rm -f "$ICON_BASE/128x128/apps/drone-ws-console.png"
rm -f "$ICON_BASE/256x256/apps/drone-ws-console.png"
rm -f "$ICON_BASE/512x512/apps/drone-ws-console.png"
rm -rf "$SHARE_DIR"
# Keep config by default; pass --purge to wipe
if [[ "${1:-}" == "--purge" ]]; then
  rm -rf "$CFG_DIR"
fi

update-desktop-database "$APP_DIR" 2>/dev/null || true
gtk-update-icon-cache -f -t "$ICON_BASE" 2>/dev/null || true
echo "Uninstalled Drone WS Console."
