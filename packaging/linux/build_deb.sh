#!/usr/bin/env bash
# Build a simple user-oriented .deb for Drone WS Console launchers.
# Requires: dpkg-deb
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
VERSION="${VERSION:-0.2.0}"
ARCH="$(dpkg --print-architecture 2>/dev/null || echo amd64)"
NAME="drone-ws-console"
ROOT="$HERE/dist/${NAME}_${VERSION}_${ARCH}"
DIST="$HERE/dist"

rm -rf "$ROOT"
mkdir -p "$ROOT/DEBIAN" \
  "$ROOT/usr/bin" \
  "$ROOT/usr/share/applications" \
  "$ROOT/usr/share/icons/hicolor/48x48/apps" \
  "$ROOT/usr/share/icons/hicolor/128x128/apps" \
  "$ROOT/usr/share/icons/hicolor/256x256/apps" \
  "$ROOT/usr/share/icons/hicolor/512x512/apps" \
  "$ROOT/usr/share/doc/$NAME" \
  "$ROOT/usr/share/$NAME"

# Anona PNG icons only (do not ship the old drawn SVG — it overrides PNG)
if [[ -f "$HERE/icons/drone-ws-console-256.png" ]]; then
  install -m 644 "$HERE/icons/drone-ws-console-256.png" \
    "$ROOT/usr/share/icons/hicolor/256x256/apps/drone-ws-console.png"
  install -m 644 "$HERE/icons/drone-ws-console-256.png" \
    "$ROOT/usr/share/icons/hicolor/128x128/apps/drone-ws-console.png"
  install -m 644 "$HERE/icons/drone-ws-console-256.png" \
    "$ROOT/usr/share/icons/hicolor/48x48/apps/drone-ws-console.png"
fi
if [[ -f "$HERE/icons/drone-ws-console.png" ]]; then
  install -m 644 "$HERE/icons/drone-ws-console.png" \
    "$ROOT/usr/share/icons/hicolor/512x512/apps/drone-ws-console.png"
fi
install -m 644 "$HERE/README.md" "$ROOT/usr/share/doc/$NAME/README.md"
cp -a "$HERE/." "$ROOT/usr/share/$NAME/"
rm -rf "$ROOT/usr/share/$NAME/dist"

cat > "$ROOT/usr/bin/drone-ws-console" <<'EOF'
#!/usr/bin/env bash
set -eo pipefail
CFG="${XDG_CONFIG_HOME:-$HOME/.config}/drone-ws/console.conf"
DRONE_WS="${DRONE_WS:-$HOME/drone_ws}"
if [[ -f "$CFG" ]]; then
  # shellcheck disable=SC1090
  set -a; source "$CFG"; set +a
  DRONE_WS="${workspace:-$DRONE_WS}"
fi
export DRONE_WS
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ -f "$ROS_SETUP" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  set -u
fi
if [[ -f "$DRONE_WS/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$DRONE_WS/install/setup.bash"
  set -u
fi
exec ros2 run drone_bringup drone_ws_console -- "$@"
EOF
chmod 755 "$ROOT/usr/bin/drone-ws-console"

cat > "$ROOT/usr/bin/drone-ws-update" <<'EOF'
#!/usr/bin/env bash
exec /usr/bin/drone-ws-console update "$@"
EOF
chmod 755 "$ROOT/usr/bin/drone-ws-update"

sed -e 's|@BIN@|/usr/bin/drone-ws-console|g' \
    -e 's|@ICON@|drone-ws-console|g' \
    "$HERE/drone-ws-console.desktop.in" > "$ROOT/usr/share/applications/drone-ws-console.desktop"

cat > "$ROOT/DEBIAN/control" <<EOF
Package: $NAME
Version: $VERSION
Section: science
Priority: optional
Architecture: $ARCH
Depends: bash, python3, python3-gi, gir1.2-gtk-3.0, gir1.2-webkit2-4.1 | gir1.2-webkit2-4.0
Maintainer: drone_ws <local@localhost>
Description: Native Linux console for drone_ws (ROS 2 sim)
 Thin desktop shell + update helpers for the drone_ws mission console.
 Requires ROS 2 Humble and a built drone_ws workspace (default ~/drone_ws).
EOF

cat > "$ROOT/DEBIAN/postinst" <<'EOF'
#!/bin/sh
set -e
CFG_DIR="${HOME:-/tmp}/.config/drone-ws"
# postinst may run as root — seed skeleton under /etc instead
mkdir -p /etc/drone-ws
if [ ! -f /etc/drone-ws/console.conf.example ]; then
  printf 'workspace=%s\nhost=127.0.0.1\nport=8765\n' "$HOME/drone_ws" \
    > /etc/drone-ws/console.conf.example 2>/dev/null || true
fi
echo "Drone WS Console installed. Bind your workspace with:"
echo "  mkdir -p ~/.config/drone-ws"
echo "  echo workspace=\$HOME/drone_ws > ~/.config/drone-ws/console.conf"
echo "  cd \$HOME/drone_ws && colcon build --symlink-install --packages-select drone_bringup"
echo "Then run: drone-ws-console"
EOF
chmod 755 "$ROOT/DEBIAN/postinst"

mkdir -p "$DIST"
dpkg-deb --build "$ROOT" "$DIST/${NAME}_${VERSION}_${ARCH}.deb"
echo "Built: $DIST/${NAME}_${VERSION}_${ARCH}.deb"
