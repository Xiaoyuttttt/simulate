#!/usr/bin/env bash
set -uo pipefail
ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
ISAAC_PY="/mnt/16T_2/txy/envs/unitree_sim_51/bin/python"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1011/gdm/Xauthority}"
mkdir -p "$ROOT/logs" "$ROOT/outputs/wasd_drive" "$ROOT/stages"
echo "DISPLAY=$DISPLAY"
echo "WASD 控制：W前进 S后退 A左转 D右转；Space停车；Esc退出"
exec "$ISAAC_PY" "$ROOT/scripts/run_straight_physics_demo.py" --config "$ROOT/config/placement.json" --wasd
