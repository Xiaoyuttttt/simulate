#!/usr/bin/env bash
set -uo pipefail

ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
PYTHON="/mnt/16T_2/txy/envs/gsplat_env/bin/python"
LOG="$ROOT/logs/09_route_picker.log"
OUT="$ROOT/outputs/route_collection"

mkdir -p "$ROOT/logs" "$OUT"
[[ -x "$PYTHON" ]] || { echo "MISSING_GSPLAT_PYTHON=$PYTHON"; exit 42; }
"$PYTHON" -c 'import numpy, PIL, plyfile; print("COLOR_MAP_ENV_PASS")' || exit $?

set +e
env -u CUDA_VISIBLE_DEVICES PYTHONUNBUFFERED=1 DISPLAY= \
timeout --foreground --signal=TERM --kill-after=30s 20m \
    "$PYTHON" "$ROOT/scripts/generate_colored_route_picker.py" \
    --config "$ROOT/config/route_map.json" 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}
set -e

required=(
    "$OUT/route_map_color.png"
    "$OUT/route_map_metadata.json"
    "$OUT/route_picker_color.html"
)
ok=1
for path in "${required[@]}"; do
    [[ -s "$path" ]] || { echo "MISSING_OUTPUT=$path"; ok=0; }
done
grep -q '^COLOR_ROUTE_PICKER_PASS$' "$LOG" || { echo "MISSING_SUCCESS_MARKER=COLOR_ROUTE_PICKER_PASS"; ok=0; }

if [[ "$rc" -eq 0 && "$ok" -eq 1 ]]; then
    echo
    echo "把下面这个单文件下载到Windows后直接双击打开："
    echo "$OUT/route_picker_color.html"
    echo "采集后导出 route_control_points.csv，再上传到服务器。"
    echo "CAMPUS_ROUTE_PICKER_PASS"
else
    echo "CAMPUS_ROUTE_PICKER_FAILED"
    [[ "$rc" -ne 0 ]] || rc=41
fi
exit "$rc"
