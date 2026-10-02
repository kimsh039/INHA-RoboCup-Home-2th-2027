#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"
python3 make_sim.py
python3 control.py &
control_pid=$!
trap 'kill "$control_pid" 2>/dev/null || true' EXIT
GZ_PARTITION=robocup_motion gz sim -r motion.world.sdf
