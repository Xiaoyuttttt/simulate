#!/usr/bin/env bash
set -uo pipefail
ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
GS_PY="/mnt/16T_2/txy/envs/gsplat_env/bin/python"
PLY="/mnt/16T_2/txy/campus_robot/assets/point_cloud.ply"
OUT="$ROOT/outputs/html_reference"
LOG="$ROOT/logs/08_html_reference_preview.log"
mkdir -p "$OUT" "$ROOT/logs"
[[ -x "$GS_PY" ]] || { echo "GSPLAT_PYTHON_MISSING=$GS_PY"; exit 81; }
[[ -s "$PLY" ]] || { echo "PLY_MISSING=$PLY"; exit 82; }
"$GS_PY" "$ROOT/scripts/export_html_reference_keyframes.py" --config "$ROOT/config/placement.json" || exit $?
set +e
CUDA_HOME=/usr/local/cuda-12.4 PATH="/usr/local/cuda-12.4/bin:$PATH" PYTHONUNBUFFERED=1 timeout --foreground --kill-after=60s 45m \
 "$GS_PY" "$ROOT/scripts/render_3dgs_camera_preview.py" --ply "$PLY" \
 --camera-path "$OUT/html_keyframes_camera_path.json" --output-dir "$OUT" \
 --max-gaussians "${MAX_GAUSSIANS:-11879665}" --device cuda:0 --preview-all-records \
 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}; set -e
if [[ "$rc" -eq 0 ]] && grep -q '^THREEDGS_CAMERA_PREVIEW_PASS$' "$LOG" && [[ -s "$OUT/camera_preview_strip.png" ]]; then
 echo "HTML_REFERENCE_PREVIEW=$OUT/camera_preview_strip.png"; echo "CAMPUS_HTML_REFERENCE_PREVIEW_PASS"; exit 0
fi
echo "CAMPUS_HTML_REFERENCE_PREVIEW_FAILED"; exit "${rc:-83}"
