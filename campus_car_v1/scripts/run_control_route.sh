#!/usr/bin/env bash
set -uo pipefail
ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
PYTHON="/mnt/16T_2/txy/envs/unitree_sim_51/bin/python"
LOG="$ROOT/logs/10_control_route.log"
GROUND="$ROOT/outputs/control_route_ground/grounded_control_route.csv"
mkdir -p "$ROOT/logs" "$ROOT/outputs/control_route_ground" "$ROOT/outputs/control_route_animation" "$ROOT/stages"
bash "$ROOT/scripts/00_check_isaac51.sh" || exit $?

set +e
env -u CUDA_VISIBLE_DEVICES PYTHONUNBUFFERED=1 DISPLAY= \
timeout --foreground --signal=TERM --kill-after=30s 30m \
  "$PYTHON" "$ROOT/scripts/ground_control_route.py" --config "$ROOT/config/placement.json" \
  2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}
set -e
if [[ "$rc" -ne 0 || ! -s "$GROUND" ]]; then
  echo "CONTROL_ROUTE_GROUND_FAILED"
  [[ "$rc" -ne 0 ]] || rc=51
  exit "$rc"
fi

set +e
env -u CUDA_VISIBLE_DEVICES PYTHONUNBUFFERED=1 DISPLAY= \
timeout --foreground --signal=TERM --kill-after=30s 20m \
  "$PYTHON" "$ROOT/scripts/build_car_animation.py" \
  --config "$ROOT/config/placement.json" --fps 15 \
  --source-csv "$GROUND" --output-subdir control_route_animation \
  --stage-name campus_car_control_route.usd 2>&1 | tee -a "$LOG"
rc=${PIPESTATUS[0]}
set -e

required=(
 "$ROOT/outputs/control_route_ground/summary.json"
 "$ROOT/outputs/control_route_ground/route_xy_check.png"
 "$ROOT/outputs/control_route_ground/route_z_profile.png"
 "$ROOT/outputs/control_route_animation/trajectory_frames.csv"
 "$ROOT/outputs/control_route_animation/animation_summary.json"
 "$ROOT/stages/campus_car_control_route.usd"
)
ok=1; for p in "${required[@]}"; do [[ -s "$p" ]] || { echo "MISSING_OUTPUT=$p"; ok=0; }; done
grep -q '^CONTROL_ROUTE_GROUND_PASS$' "$LOG" || ok=0
grep -q '^CAR_ANIMATION_PASS$' "$LOG" || ok=0
if [[ "$rc" -eq 0 && "$ok" -eq 1 ]]; then echo "CAMPUS_CONTROL_ROUTE_PASS"; else echo "CAMPUS_CONTROL_ROUTE_FAILED"; [[ "$rc" -ne 0 ]] || rc=52; fi
exit "$rc"
