#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"
# Keep Gazebo discovery on this PC, avoiding other simulations on the LAN.
export GZ_IP="${GZ_IP:-127.0.0.1}"
python3 make_sim.py "$@"
python3 ../tools/control.py &
control_pid=$!
trap 'kill "$control_pid" 2>/dev/null || true' EXIT
if nvidia-smi >/dev/null 2>&1; then
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
fi
GZ_PARTITION=robocup_motion gz sim -r build/motion.world.sdf
