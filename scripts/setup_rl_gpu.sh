#!/usr/bin/env bash
# Safe GPU setup for Path G on AMD iGPU + NVIDIA dGPU laptops (ASUS TUF, etc.).
#
# IMPORTANT (hybrid laptops):
#   Do NOT force-load nvidia_drm / nvidia_modeset for training.
#   That can steal the display from amdgpu and cause a black screen.
#   We only need nvidia + nvidia_uvm for CUDA compute.
#
# If the screen went black after an earlier modprobe:
#   1) Switch to a TTY (Ctrl+Alt+F3), log in
#   2) sudo prime-select on-demand
#   3) sudo reboot
#   Display should come back on the AMD iGPU; CUDA can still work after login.
set -euo pipefail

KVER="$(uname -r)"
echo "Kernel: $KVER"

if ! python3 -c 'import torch; raise SystemExit(0 if torch.cuda.is_available() else 1)' 2>/dev/null; then
  echo "CUDA not visible yet — restoring NVIDIA *compute* packages (not display)..."
  sudo apt-get update
  if apt-cache show "linux-modules-nvidia-595-${KVER}" >/dev/null 2>&1; then
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
      "linux-modules-nvidia-595-${KVER}" \
      nvidia-kernel-common-595 \
      libnvidia-compute-595 \
      nvidia-utils-595 || true
  fi
  if ! modinfo nvidia >/dev/null 2>&1; then
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
      nvidia-driver-570 nvidia-dkms-570 nvidia-utils-570
  fi
  # Compute only — avoid nvidia_drm (display)
  sudo modprobe nvidia || true
  sudo modprobe nvidia_uvm || true
  # Keep prime on-demand so AMD keeps the desktop
  if command -v prime-select >/dev/null; then
    sudo prime-select on-demand || true
  fi
fi

python3 - <<'PY'
import torch
print('torch', torch.__version__)
print('cuda_built', torch.version.cuda)
print('available', torch.cuda.is_available())
if not torch.cuda.is_available():
    raise SystemExit(
        'Still no CUDA. Reboot once with: sudo reboot\n'
        'Then: python3 -c "import torch; print(torch.cuda.is_available())"'
    )
print('device', torch.cuda.get_device_name(0))
PY

echo
echo "GPU OK. Train with:"
echo "  cd ~/drone_ws"
echo "  PYTHONPATH=src/drone_rl_planner:\$PYTHONPATH \\"
echo "    python3 -m drone_rl_planner.train_sb3_ppo --fresh --device cuda --target 0.95"
