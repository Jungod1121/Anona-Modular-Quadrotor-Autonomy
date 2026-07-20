# Drone WS Console — Linux desktop app

Turns the mission console into a **real Linux application** (app menu + native
window), while keeping **all** dashboard functions (start/stop sim, planners,
maps, multi-agent, acceptance, RL, logs).

This is **not** “open Chrome and pin a tab”. The app:

1. Sources ROS 2 + your `drone_ws`
2. Starts the same `dashboard` backend (full API)
3. Opens a **native window** (GTK WebKit; falls back to browser only if needed)

## Why updates stay easy

The app is a **thin shell** bound to your workspace path
(`~/.config/drone-ws/console.conf`). Your code stays in `drone_ws`.

| You change… | How to refresh the app |
|-------------|------------------------|
| UI (`dashboard_static/*`) with `--symlink-install` | Restart the app |
| Python in `drone_bringup` | `drone-ws-update` (or restart after rebuild) |
| Broader packages / after `git pull` | `drone-ws-update` |

```bash
drone-ws-update
# same as:
drone-ws-console update
```

That runs `git pull` (if git) + `colcon build --symlink-install --packages-up-to drone_bringup drone_visualization`.

## Install (recommended)

```bash
# once: build package
source /opt/ros/humble/setup.bash
cd ~/drone_ws && colcon build --symlink-install --packages-select drone_bringup

# install app into ~/.local (no root)
bash ~/drone_ws/packaging/linux/install.sh

# native window deps (Ubuntu)
sudo apt install -y python3-gi gir1.2-gtk-3.0 gir1.2-webkit2-4.1
```

Then open **Activities → 无人机仿真控制台 / Drone WS Console**, or:

```bash
drone-ws-console          # run
drone-ws-console info     # show workspace binding
drone-ws-update           # pull + rebuild
```

## Uninstall

```bash
bash ~/drone_ws/packaging/linux/uninstall.sh
bash ~/drone_ws/packaging/linux/uninstall.sh --purge   # also remove ~/.config/drone-ws
```

## Optional `.deb`

```bash
bash ~/drone_ws/packaging/linux/build_deb.sh
sudo apt install ./packaging/linux/dist/drone-ws-console_*.deb
```

The `.deb` installs the same launchers system-wide and still points at your
`DRONE_WS` (default `~/drone_ws`). It does **not** vendor ROS — ROS 2 Humble
must already be installed.

## Architecture

```
┌─────────────────────────────┐
│  Drone WS Console (window)  │  GTK WebKit / pywebview
└──────────────┬──────────────┘
               │ http://127.0.0.1:8765
┌──────────────▼──────────────┐
│  dashboard_server (backend) │  same APIs as the website
└──────────────┬──────────────┘
               │ ros2 launch / topics
┌──────────────▼──────────────┐
│  drone_ws workspace + ROS 2 │  updated via drone-ws-update
└─────────────────────────────┘
```
