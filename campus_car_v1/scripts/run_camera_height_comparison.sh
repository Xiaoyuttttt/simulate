#!/usr/bin/env bash
set -uo pipefail
ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
GS_PY="/mnt/16T_2/txy/envs/gsplat_env/bin/python"
PLY="/mnt/16T_2/txy/campus_robot/assets/point_cloud.ply"
OUT="$ROOT/outputs/camera_height_comparison"
CAMERA="$OUT/camera_path.json"
LOG="$ROOT/logs/14_camera_height_comparison.log"
mkdir -p "$OUT" "$ROOT/logs"
[[ -x "$GS_PY" && -s "$PLY" ]] || { echo "HEIGHT_COMPARISON_INPUT_MISSING"; exit 101; }
"$GS_PY" "$ROOT/scripts/export_camera_height_comparison.py" --config "$ROOT/config/placement.json" --output "$CAMERA" --width 1280 --height 720 || exit $?
set +e
CUDA_HOME=/usr/local/cuda-12.4 PATH="/usr/local/cuda-12.4/bin:$PATH" PYTHONUNBUFFERED=1 timeout --foreground --signal=TERM --kill-after=60s 3h \
 "$GS_PY" "$ROOT/scripts/render_3dgs_camera_preview.py" --ply "$PLY" --camera-path "$CAMERA" --output-dir "$OUT" \
 --max-gaussians 0 --route-crop-margin "${ROUTE_CROP_MARGIN:-120}" --device cuda:0 --preview-all-records 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}; set -e
required=("$OUT/camera_preview_strip.png" "$OUT/summary.json")
ok=1; for p in "${required[@]}"; do [[ -s "$p" ]] || { echo "MISSING_OUTPUT=$p"; ok=0; }; done
grep -q '^THREEDGS_CAMERA_PREVIEW_PASS$' "$LOG" || ok=0
if [[ "$rc" -eq 0 && "$ok" -eq 1 ]]; then echo "CAMPUS_CAMERA_HEIGHT_COMPARISON_PASS"; else echo "CAMPUS_CAMERA_HEIGHT_COMPARISON_FAILED"; [[ "$rc" -ne 0 ]] || rc=102; fi
exit "$rc"
