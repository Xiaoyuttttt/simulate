#!/usr/bin/env bash
set -euo pipefail
ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
ISAAC_PY="/mnt/16T_2/txy/envs/unitree_sim_51/bin/python"
mkdir -p "$ROOT/logs" "$ROOT/outputs/preset_drive" "$ROOT/stages"
env -u DISPLAY -u XAUTHORITY PYTHONUNBUFFERED=1 \
  "$ISAAC_PY" "$ROOT/scripts/run_straight_physics_demo.py" \
  --config "$ROOT/config/placement.json" --preset \
  2>&1 | tee "$ROOT/logs/preset_drive.log"
"$ISAAC_PY" "$ROOT/scripts/render_turn_diagnostic_video.py" \
  --source "$ROOT/outputs/preset_drive/trajectory_frames.csv" \
  --output "$ROOT/outputs/preset_drive/preset_drive.mp4"
