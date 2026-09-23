#!/usr/bin/env bash
set -uo pipefail

ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
PYTHON="/mnt/16T_2/txy/envs/unitree_sim_51/bin/python"
LOG="$ROOT/logs/02_ground_waypoints.log"
OUT="$ROOT/outputs/ground_waypoints"

mkdir -p "$ROOT/logs" "$OUT" "$ROOT/stages"
bash "$ROOT/scripts/00_check_isaac51.sh" || exit $?

set +e
env -u CUDA_VISIBLE_DEVICES PYTHONUNBUFFERED=1 DISPLAY= \
timeout --foreground --signal=TERM --kill-after=30s 25m \
    "$PYTHON" "$ROOT/scripts/ground_all_waypoints.py" --config "$ROOT/config/placement.json" \
    2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}
set -e

required=(
    "$OUT/ground_waypoints.csv"
    "$OUT/ground_waypoints_diagnostics.json"
    "$OUT/route_overview.png"
    "$OUT/route_side.png"
    "$ROOT/stages/campus_ground_waypoints.usd"
)
ok=1
for path in "${required[@]}"; do
    [[ -s "$path" ]] || { echo "MISSING_OUTPUT=$path"; ok=0; }
done
grep -q '^GROUND_WAYPOINTS_PASS$' "$LOG" || { echo "MISSING_SUCCESS_MARKER=GROUND_WAYPOINTS_PASS"; ok=0; }

if [[ "$rc" -eq 0 && "$ok" -eq 1 ]]; then
    echo "CAMPUS_GROUND_WAYPOINTS_PASS"
else
    echo "CAMPUS_GROUND_WAYPOINTS_FAILED"
    [[ "$rc" -ne 0 ]] || rc=31
fi
exit "$rc"
