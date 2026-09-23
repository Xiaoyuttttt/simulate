#!/usr/bin/env bash
set -uo pipefail
ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
PYTHON="/mnt/16T_2/txy/envs/unitree_sim_51/bin/python"
LOG="$ROOT/logs/03_car_animation.log"
OUT="$ROOT/outputs/car_animation"
mkdir -p "$ROOT/logs" "$OUT" "$ROOT/stages"
bash "$ROOT/scripts/00_check_isaac51.sh" || exit $?
set +e
env -u CUDA_VISIBLE_DEVICES PYTHONUNBUFFERED=1 DISPLAY= timeout --foreground --signal=TERM --kill-after=30s 10m \
  "$PYTHON" "$ROOT/scripts/build_car_animation.py" --config "$ROOT/config/placement.json" --fps 15 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}
set -e
required=("$OUT/trajectory_frames.csv" "$OUT/animation_summary.json" "$ROOT/stages/campus_car_animated.usd")
ok=1
for path in "${required[@]}"; do [[ -s "$path" ]] || { echo "MISSING_OUTPUT=$path"; ok=0; }; done
grep -q '^CAR_ANIMATION_PASS$' "$LOG" || { echo "MISSING_SUCCESS_MARKER=CAR_ANIMATION_PASS"; ok=0; }
if [[ "$rc" -eq 0 && "$ok" -eq 1 ]]; then echo "CAMPUS_CAR_ANIMATION_PASS"; else echo "CAMPUS_CAR_ANIMATION_FAILED"; [[ "$rc" -ne 0 ]] || rc=41; fi
exit "$rc"
