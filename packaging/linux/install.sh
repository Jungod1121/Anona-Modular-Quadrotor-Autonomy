#!/usr/bin/env bash
# Install Drone WS Console as a real user-local Linux application.
# - App menu entry + native window launcher
# - Update command wired to this workspace (git pull + colcon build)
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS="$(cd "$HERE/../.." && pwd)"
export DRONE_WS="${DRONE_WS:-$WS}"

BIN_DIR="${XDG_BIN_HOME:-$HOME/.local/bin}"
APP_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
ICON_BASE="${XDG_DATA_HOME:-$HOME/.local/share}/icons/hicolor"
ICON_SVG="$ICON_BASE/scalable/apps"
ICON_256="$ICON_BASE/256x256/apps"
ICON_512="$ICON_BASE/512x512/apps"
CFG_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/drone-ws"
SHARE_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/drone-ws-console"

mkdir -p "$BIN_DIR" "$APP_DIR" "$ICON_SVG" "$ICON_256" "$ICON_512" "$CFG_DIR" "$SHARE_DIR"

# System deps hint (non-fatal)
need_pkgs=()
python3 -c "import gi; gi.require_version('Gtk','3.0'); gi.require_version('WebKit2','4.1')" 2>/dev/null \
  || python3 -c "import gi; gi.require_version('Gtk','3.0'); gi.require_version('WebKit2','4.0')" 2>/dev/null \
  || need_pkgs+=(gir1.2-webkit2-4.1 gir1.2-gtk-3.0 python3-gi)
if ((${#need_pkgs[@]})); then
  echo "NOTE: for a native window (not browser), install:"
  echo "  sudo apt install ${need_pkgs[*]}"
fi

# App icon: yellow mascot PNG (do not ship old SVG — it overrides PNG in many DEs)
rm -f "$ICON_SVG/drone-ws-console.svg"
mkdir -p "$ICON_BASE/48x48/apps" "$ICON_BASE/128x128/apps" \
  "$ICON_BASE/64x64/apps" "$ICON_BASE/32x32/apps" \
  "${XDG_DATA_HOME:-$HOME/.local/share}/pixmaps"
ICON_SRC=""
if [[ -f "$HERE/icons/drone-ws-console-mascot.png" ]]; then
  ICON_SRC="$HERE/icons/drone-ws-console-mascot.png"
elif [[ -f "$HERE/icons/drone-ws-console.png" ]]; then
  ICON_SRC="$HERE/icons/drone-ws-console.png"
fi
if [[ -n "$ICON_SRC" ]]; then
  install -m 644 "$ICON_SRC" "$ICON_512/drone-ws-console.png"
  install -m 644 "$ICON_SRC" "$ICON_512/drone-ws-console-mascot.png"
  install -m 644 "$ICON_SRC" "${XDG_DATA_HOME:-$HOME/.local/share}/pixmaps/drone-ws-console.png"
fi
if [[ -f "$HERE/icons/drone-ws-console-256.png" ]]; then
  install -m 644 "$HERE/icons/drone-ws-console-256.png" "$ICON_256/drone-ws-console.png"
  # Scale-friendly copies for menu/taskbar themes that look for 48/128
  if command -v convert >/dev/null 2>&1; then
    convert "$HERE/icons/drone-ws-console-256.png" -resize 128x128 \
      "$ICON_BASE/128x128/apps/drone-ws-console.png"
    convert "$HERE/icons/drone-ws-console-256.png" -resize 64x64 \
      "$ICON_BASE/64x64/apps/drone-ws-console.png"
    convert "$HERE/icons/drone-ws-console-256.png" -resize 48x48 \
      "$ICON_BASE/48x48/apps/drone-ws-console.png"
    convert "$HERE/icons/drone-ws-console-256.png" -resize 32x32 \
      "$ICON_BASE/32x32/apps/drone-ws-console.png"
  else
    install -m 644 "$HERE/icons/drone-ws-console-256.png" \
      "$ICON_BASE/128x128/apps/drone-ws-console.png"
    install -m 644 "$HERE/icons/drone-ws-console-256.png" \
      "$ICON_BASE/48x48/apps/drone-ws-console.png"
  fi
fi
cp -a "$HERE/." "$SHARE_DIR/"

# Persistent config → this workspace
cat > "$CFG_DIR/console.conf" <<EOF
workspace=$DRONE_WS
host=127.0.0.1
port=8765
EOF

# Launcher: sources ROS + workspace, then runs desktop app
cat > "$BIN_DIR/drone-ws-console" <<EOF
#!/usr/bin/env bash
set -eo pipefail
export DRONE_WS="$DRONE_WS"
CFG="\${XDG_CONFIG_HOME:-\$HOME/.config}/drone-ws/console.conf"
if [[ -f "\$CFG" ]]; then
  # shellcheck disable=SC1090
  set -a; source "\$CFG"; set +a
  export DRONE_WS="\${workspace:-\$DRONE_WS}"
fi
ROS_SETUP="\${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ -f "\$ROS_SETUP" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "\$ROS_SETUP"
  set -u
fi
if [[ -f "\$DRONE_WS/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "\$DRONE_WS/install/setup.bash"
  set -u
fi
exec ros2 run drone_bringup drone_ws_console -- "\$@"
EOF
chmod 755 "$BIN_DIR/drone-ws-console"

# Friendly alias
ln -sfn "$BIN_DIR/drone-ws-console" "$BIN_DIR/drone-ws-console-update" 2>/dev/null || true
# update helper always runs update subcommand
cat > "$BIN_DIR/drone-ws-update" <<EOF
#!/usr/bin/env bash
exec "$BIN_DIR/drone-ws-console" update "\$@"
EOF
chmod 755 "$BIN_DIR/drone-ws-update"

sed -e "s|@BIN@|$BIN_DIR/drone-ws-console|g" \
    -e "s|@ICON@|$ICON_512/drone-ws-console-mascot.png|g" \
    "$HERE/drone-ws-console.desktop.in" > "$APP_DIR/drone-ws-console.desktop"
chmod 644 "$APP_DIR/drone-ws-console.desktop"

# Optional Actions for right-click Update
cat >> "$APP_DIR/drone-ws-console.desktop" <<EOF

Actions=Update;Info;

[Desktop Action Update]
Name=Update from workspace
Name[zh_CN]=从工作空间更新
Exec=$BIN_DIR/drone-ws-update

[Desktop Action Info]
Name=Show info
Name[zh_CN]=显示信息
Exec=$BIN_DIR/drone-ws-console info
EOF

update-desktop-database "$APP_DIR" 2>/dev/null || true
gtk-update-icon-cache -f -t "${XDG_DATA_HOME:-$HOME/.local/share}/icons/hicolor" 2>/dev/null || true

echo
echo "Installed Drone WS Console (Linux app)"
echo "  Menu:      Drone WS Console / 无人机仿真控制台"
echo "  Command:   drone-ws-console"
echo "  Update:    drone-ws-update   (or: drone-ws-console update)"
echo "  Workspace: $DRONE_WS"
echo "  Config:    $CFG_DIR/console.conf"
echo
if [[ ! -f "$DRONE_WS/install/setup.bash" ]]; then
  echo "Workspace not built yet — building drone_bringup…"
  # shellcheck disable=SC1090
  source /opt/ros/humble/setup.bash
  (cd "$DRONE_WS" && colcon build --symlink-install --packages-select drone_bringup)
fi
echo "Done. Open from Activities, or run: drone-ws-console"
