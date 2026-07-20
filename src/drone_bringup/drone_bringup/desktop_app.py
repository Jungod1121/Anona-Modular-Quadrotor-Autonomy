#!/usr/bin/env python3
"""Native Linux desktop shell for the drone_ws mission console.

This is not a browser bookmark: it owns a dedicated window, runs the same
dashboard backend (all Start/Stop/ROS/acceptance APIs), and can update from
the workspace via `drone-ws-console update`.
"""

from __future__ import annotations

import argparse
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import ThreadingHTTPServer
from pathlib import Path
from typing import Optional


CONFIG_DIR = Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'drone-ws'
CONFIG_FILE = CONFIG_DIR / 'console.conf'


def _read_conf() -> dict:
    conf = {}
    if CONFIG_FILE.is_file():
        for line in CONFIG_FILE.read_text(encoding='utf-8').splitlines():
            line = line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            k, v = line.split('=', 1)
            conf[k.strip()] = v.strip()
    return conf


def _write_conf(**kwargs: str) -> None:
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    conf = _read_conf()
    conf.update({k: str(v) for k, v in kwargs.items()})
    lines = [f'{k}={conf[k]}' for k in sorted(conf)]
    CONFIG_FILE.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def _default_workspace() -> Path:
    conf = _read_conf()
    if conf.get('workspace'):
        return Path(conf['workspace']).expanduser().resolve()
    env = os.environ.get('DRONE_WS')
    if env:
        return Path(env).expanduser().resolve()
    # desktop_app lives in src/drone_bringup/drone_bringup/
    here = Path(__file__).resolve()
    candidate = here.parents[3]  # .../drone_ws
    if (candidate / 'install' / 'setup.bash').is_file() or (candidate / 'src').is_dir():
        return candidate
    return Path.home() / 'drone_ws'


def _port_open(host: str, port: int) -> bool:
    try:
        with socket.create_connection((host, port), timeout=0.35):
            return True
    except OSError:
        return False


def _server_supports_map_api(host: str, port: int) -> bool:
    """True if the process on host:port serves the current map occupancy API."""
    try:
        import urllib.error
        import urllib.request
        url = f'http://{host}:{port}/api/map/occupancy'
        with urllib.request.urlopen(url, timeout=1.2) as resp:
            if getattr(resp, 'status', 200) != 200:
                return False
            body = resp.read(256).decode('utf-8', errors='ignore')
            return '"ok"' in body
    except Exception:
        return False


def _free_port(port: int) -> None:
    """Best-effort: stop whatever is listening on the dashboard port."""
    for cmd in (
        ['fuser', '-k', f'{port}/tcp'],
        ['pkill', '-f', 'lib/drone_bringup/dashboard'],
    ):
        try:
            subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)
        except Exception:
            pass
    deadline = time.time() + 3.0
    while time.time() < deadline:
        if not _port_open('127.0.0.1', port):
            return
        time.sleep(0.15)


def _pick_port(host: str, preferred: int) -> int:
    """Return preferred port if free/usable, else the next free port in range."""
    candidates = [preferred] + [p for p in range(preferred + 1, preferred + 8)
                                if p != preferred]
    for port in candidates:
        if not _port_open(host, port):
            return port
        if _server_supports_map_api(host, port):
            return port
        _free_port(port)
        if not _port_open(host, port):
            return port
    raise RuntimeError(
        f'Ports {preferred}–{preferred + 7} are busy with outdated dashboards.\n'
        f'Run once:\n'
        f'  sudo fuser -k {preferred}/tcp\n'
        f'  pkill -f lib/drone_bringup/dashboard || true')


def _wait_http(host: str, port: int, timeout: float = 12.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _port_open(host, port):
            return True
        time.sleep(0.15)
    return False


def _source_and_run_update(workspace: Path) -> int:
    """Pull + rebuild workspace packages used by the console."""
    script = f'''
set -eo pipefail
cd "{workspace}"
if [[ -d .git ]]; then
  git pull --ff-only || git pull
fi
# ROS setup.bash references optional unset vars; don't use nounset while sourcing.
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
if [[ -f install/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source install/setup.bash
  set -u
fi
colcon build --symlink-install --packages-up-to drone_bringup drone_visualization
echo "UPDATE_OK workspace={workspace}"
'''
    print(f'Updating console from workspace: {workspace}', flush=True)
    proc = subprocess.run(['bash', '-lc', script], check=False)
    if proc.returncode == 0:
        sha = ''
        try:
            sha = subprocess.check_output(
                ['git', '-C', str(workspace), 'rev-parse', '--short', 'HEAD'],
                text=True).strip()
        except Exception:
            pass
        _write_conf(workspace=str(workspace), last_update=time.strftime('%Y-%m-%dT%H:%M:%S'),
                    git_sha=sha or 'unknown')
        print('Update finished. Restart Drone WS Console to load changes.', flush=True)
    return int(proc.returncode)


def _start_server(host: str, port: int, workspace: Path) -> tuple[ThreadingHTTPServer, threading.Thread, bool]:
    """Return (httpd, thread, owned). owned=False if we attached to an existing server."""
    if _port_open(host, port):
        if _server_supports_map_api(host, port):
            print(f'Using existing dashboard at http://{host}:{port}/', flush=True)
            return None, None, False  # type: ignore[return-value]
        print(
            f'Existing dashboard on :{port} is outdated or broken; restarting…',
            flush=True)
        _free_port(port)

    from drone_bringup.dashboard_server import create_httpd, serve_httpd

    httpd = create_httpd(host, port, str(workspace))
    thread = threading.Thread(target=serve_httpd, args=(httpd,), name='drone-dash', daemon=True)
    thread.start()
    if not _wait_http(host, port):
        raise RuntimeError(f'Dashboard failed to start on {host}:{port}')
    print(f'Dashboard backend: http://{host}:{port}/', flush=True)
    return httpd, thread, True


def _stop_server(httpd: Optional[ThreadingHTTPServer]) -> None:
    if httpd is None:
        return
    try:
        httpd.shutdown()
    except Exception:
        pass


def _resolve_app_icon() -> Optional[Path]:
    """Yellow mascot PNG for the native window / taskbar / app menu."""
    pkg = Path(__file__).resolve().parent
    ws = Path(__file__).resolve().parents[3]
    xdg = Path(os.environ.get('XDG_DATA_HOME', Path.home() / '.local' / 'share'))
    candidates = [
        xdg / 'icons' / 'hicolor' / '512x512' / 'apps' / 'drone-ws-console-mascot.png',
        xdg / 'icons' / 'hicolor' / '512x512' / 'apps' / 'drone-ws-console.png',
        xdg / 'pixmaps' / 'drone-ws-console.png',
        ws / 'packaging' / 'linux' / 'icons' / 'drone-ws-console-mascot.png',
        ws / 'packaging' / 'linux' / 'icons' / 'drone-ws-console.png',
        pkg / 'dashboard_static' / 'brand-mascot.png',
    ]
    for path in candidates:
        if path.is_file():
            return path
    return None


def _open_gtk_window(url: str, title: str) -> int:
    import gi
    gi.require_version('Gtk', '3.0')
    # Prefer 4.1 on newer Ubuntu, fall back to 4.0
    try:
        gi.require_version('WebKit2', '4.1')
    except ValueError:
        gi.require_version('WebKit2', '4.0')
    from gi.repository import Gtk, WebKit2, GdkPixbuf, Gdk

    try:
        Gdk.set_program_class('drone-ws-console')
    except Exception:
        pass

    win = Gtk.Window(title=title)
    win.set_default_size(1440, 920)
    try:
        win.set_wmclass('drone-ws-console', 'drone-ws-console')
    except Exception:
        pass
    win.connect('destroy', Gtk.main_quit)
    icon = _resolve_app_icon()
    if icon is not None:
        try:
            pix = GdkPixbuf.Pixbuf.new_from_file(str(icon))
            # Multi-size list so taskbar / alt-tab pick a sharp yellow mascot.
            icons = []
            for edge in (512, 256, 128, 64, 48, 32):
                try:
                    icons.append(pix.scale_simple(edge, edge, GdkPixbuf.InterpType.BILINEAR))
                except Exception:
                    pass
            if icons:
                Gtk.Window.set_default_icon_list(icons)
                win.set_icon_list(icons)
            else:
                win.set_icon(pix)
                Gtk.Window.set_default_icon(pix)
            print(f'Window icon: {icon}', flush=True)
        except Exception as exc:
            print(f'Could not set window icon ({exc})', flush=True)

    view = WebKit2.WebView()
    # LiquidGlass needs WebGL + compositing (default WebKit settings).
    try:
        settings = view.get_settings()
        settings.set_enable_webgl(True)
        settings.set_hardware_acceleration_policy(
            WebKit2.HardwareAccelerationPolicy.ALWAYS)
    except Exception:
        pass

    # target="_blank" / window.open would otherwise open an empty window (or nothing).
    def _on_create(_web_view, navigation_action):
        req = navigation_action.get_request()
        uri = req.get_uri() if req is not None else None
        if uri:
            view.load_uri(uri)
        return None

    def _on_decide_policy(_web_view, decision, decision_type):
        if decision_type == WebKit2.PolicyDecisionType.NEW_WINDOW_ACTION:
            try:
                nav = decision.get_navigation_action()
                req = nav.get_request() if nav is not None else None
                uri = req.get_uri() if req is not None else None
                if uri:
                    view.load_uri(uri)
                decision.ignore()
                return True
            except Exception:
                return False
        return False

    view.connect('create', _on_create)
    view.connect('decide-policy', _on_decide_policy)
    view.load_uri(url)
    win.add(view)
    win.show_all()
    Gtk.main()
    return 0


def _open_pywebview(url: str, title: str) -> int:
    import webview  # type: ignore
    webview.create_window(title, url, width=1440, height=920)
    webview.start()
    return 0


def _open_window(url: str, title: str) -> int:
    # 1) GTK + WebKit (system packages, true desktop window)
    try:
        return _open_gtk_window(url, title)
    except Exception as exc:
        print(f'GTK WebKit unavailable ({exc}); trying pywebview…', flush=True)
    # 2) pywebview
    try:
        return _open_pywebview(url, title)
    except Exception as exc:
        print(f'pywebview unavailable ({exc}); opening system browser…', flush=True)
    # 3) last resort
    webbrowser.open(url)
    print('Running in browser fallback. Press Ctrl+C to stop the backend.', flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        return 0


def run_app(host: str, port: int, workspace: Path) -> int:
    try:
        port = _pick_port(host, port)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    _write_conf(workspace=str(workspace), host=host, port=str(port))
    httpd = None
    owned = False
    try:
        httpd, _thread, owned = _start_server(host, port, workspace)
    except OSError as exc:
        print(f'Cannot start dashboard: {exc}', file=sys.stderr)
        return 1
    except Exception as exc:
        print(f'Cannot start dashboard: {exc}', file=sys.stderr)
        return 1

    url = f'http://{host}:{port}/'
    if port != int(_read_conf().get('port', port)):
        pass
    print(f'Console UI: {url}', flush=True)
    title = '无人机仿真控制台 · Flight Deck'

    def _sig(_signum, _frame):
        _stop_server(httpd if owned else None)
        sys.exit(0)

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    try:
        return _open_window(url, title)
    finally:
        if owned:
            _stop_server(httpd)


def main(argv: Optional[list] = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0].startswith('-'):
        argv = ['run', *argv]

    parser = argparse.ArgumentParser(
        description='Drone WS Console — native Linux app for the sim mission console')
    sub = parser.add_subparsers(dest='cmd', required=True)

    p_run = sub.add_parser('run', help='Start the desktop console (default)')
    p_run.add_argument('--host', default=None)
    p_run.add_argument('--port', type=int, default=None)
    p_run.add_argument('--workspace', default=None)

    p_up = sub.add_parser('update', help='Pull workspace + rebuild packages used by the console')
    p_up.add_argument('--workspace', default=None)

    sub.add_parser('info', help='Show installed workspace / config')

    args = parser.parse_args(argv)

    conf = _read_conf()
    workspace = Path(
        getattr(args, 'workspace', None)
        or conf.get('workspace')
        or _default_workspace()
    ).expanduser().resolve()

    if args.cmd == 'info':
        print(f'config:    {CONFIG_FILE}')
        print(f'workspace: {workspace}')
        print(f'host:      {conf.get("host", "127.0.0.1")}')
        print(f'port:      {conf.get("port", "8765")}')
        print(f'git_sha:   {conf.get("git_sha", "—")}')
        print(f'updated:   {conf.get("last_update", "—")}')
        return 0

    if args.cmd == 'update':
        return _source_and_run_update(workspace)

    host = args.host or conf.get('host') or '127.0.0.1'
    port = int(args.port or conf.get('port') or 8765)
    if not (workspace / 'install' / 'setup.bash').is_file():
        print(
            f'Workspace not built: {workspace}\n'
            f'Run: cd "{workspace}" && colcon build --symlink-install\n'
            f'Or:  drone-ws-console update',
            file=sys.stderr)
        return 1
    return run_app(host, port, workspace)


if __name__ == '__main__':
    sys.exit(main())
