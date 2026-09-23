#!/usr/bin/env bash
set -uo pipefail

ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
ISAAC_PY="/mnt/16T_2/txy/envs/unitree_sim_51/bin/python"
LOG="$ROOT/logs/18_turn_physics_demo.log"
mkdir -p "$ROOT/logs" "$ROOT/outputs/turn_physics_demo" "$ROOT/stages"

set +e
env -u CUDA_VISIBLE_DEVICES PYTHONUNBUFFERED=1 DISPLAY= \
  timeout --foreground --signal=TERM --kill-after=30s 20m \
  "$ISAAC_PY" "$ROOT/scripts/run_straight_physics_demo.py" \
  --config "$ROOT/config/placement.json" --turn-test 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}
set -e

if [[ "$rc" -ne 0 ]] || ! grep -q '^STRAIGHT_PHYSICS_DEMO_PASS$' "$LOG"; then
  echo "CAMPUS_TURN_PHYSICS_DEMO_FAILED"
  [[ "$rc" -ne 0 ]] || rc=122
  exit "$rc"
fi
echo "CAMPUS_TURN_PHYSICS_DEMO_PASS"
