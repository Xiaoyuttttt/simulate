#!/usr/bin/env bash
set -uo pipefail
ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
PYTHON="/mnt/16T_2/txy/envs/unitree_sim_51/bin/python"
LOG="$ROOT/logs/11_control_route_preview.log"
OUT="$ROOT/outputs/control_route_preview"
mkdir -p "$ROOT/logs" "$OUT"
bash "$ROOT/scripts/00_check_isaac51.sh" || exit $?
set +e
env -u CUDA_VISIBLE_DEVICES PYTHONUNBUFFERED=1 DISPLAY= timeout --foreground --signal=TERM --kill-after=30s 25m \
 "$PYTHON" "$ROOT/scripts/render_control_route_preview.py" --config "$ROOT/config/placement.json" --views 9 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}; set -e
required=("$OUT/control_route_preview_sheet.png" "$OUT/summary.json")
ok=1; for p in "${required[@]}"; do [[ -s "$p" ]] || { echo "MISSING_OUTPUT=$p"; ok=0; }; done
grep -q '^CONTROL_ROUTE_PREVIEW_PASS$' "$LOG" || ok=0
if [[ "$rc" -eq 0 && "$ok" -eq 1 ]]; then echo "CAMPUS_CONTROL_ROUTE_PREVIEW_PASS"; else echo "CAMPUS_CONTROL_ROUTE_PREVIEW_FAILED"; [[ "$rc" -ne 0 ]] || rc=61; fi
exit "$rc"
