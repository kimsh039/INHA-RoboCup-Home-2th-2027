#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"
# 같은 네트워크의 다른 PC 시뮬레이션과 섞이지 않도록 Gazebo Transport를 이 PC 안으로 제한
export GZ_IP="${GZ_IP:-127.0.0.1}"
python3 build.py
python3 make_sim.py
python3 control.py &
control_pid=$!
trap 'kill "$control_pid" 2>/dev/null || true' EXIT
# NVIDIA 드라이버가 있으면 PRIME on-demand 환경에서 Gazebo 렌더링을 외장 GPU로 보냄
if nvidia-smi >/dev/null 2>&1; then
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
fi
GZ_PARTITION=robocup_motion gz sim -r motion.world.sdf
