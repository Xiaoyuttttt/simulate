#!/usr/bin/env bash
set -uo pipefail
ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
GS_PY="/mnt/16T_2/txy/envs/gsplat_env/bin/python"
SOURCE_PLY="/mnt/16T_2/txy/campus_robot/assets/point_cloud.ply"
OUT="$ROOT/outputs/high_camera_small_region"
CAMERA="$OUT/high_camera_path.json"
MARGIN="${REGION_MARGIN:-40}"
CROPPED="$OUT/campus_route_region_${MARGIN}m.ply"
CROP_SUMMARY="$OUT/campus_route_region_${MARGIN}m.summary.json"
LOG="$ROOT/logs/15_high_camera_small_region.log"
mkdir -p "$OUT" "$ROOT/logs"
[[ -x "$GS_PY" && -s "$SOURCE_PLY" ]] || { echo "HIGH_CAMERA_INPUT_MISSING"; exit 101; }

echo "[1/3] 生成5个HIGH相机：离地4.00m，向下18度"
"$GS_PY" "$ROOT/scripts/export_camera_height_comparison.py" --config "$ROOT/config/placement.json" \
 --output "$CAMERA" --width 1280 --height 720 --variant high || exit $?

echo "[2/3] 从完整PLY无损裁剪路线周围${MARGIN}m区域"
"$GS_PY" "$ROOT/scripts/crop_3dgs_route_region.py" --input "$SOURCE_PLY" --camera-path "$CAMERA" \
 --output "$CROPPED" --margin "$MARGIN" || exit $?

echo "[3/3] 使用裁剪后的全部高斯渲染5张HIGH"
set +e
CUDA_HOME=/usr/local/cuda-12.4 PATH="/usr/local/cuda-12.4/bin:$PATH" PYTHONUNBUFFERED=1 timeout --foreground --signal=TERM --kill-after=60s 3h \
 "$GS_PY" "$ROOT/scripts/render_3dgs_camera_preview.py" --ply "$CROPPED" --camera-path "$CAMERA" \
 --output-dir "$OUT" --max-gaussians 0 --route-crop-margin 0 --preserve-all-valid --device cuda:0 --preview-all-records 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}; set -e
required=("$CROPPED" "$CROP_SUMMARY" "$OUT/camera_preview_strip.png" "$OUT/summary.json")
ok=1; for path in "${required[@]}"; do [[ -s "$path" ]] || { echo "MISSING_OUTPUT=$path"; ok=0; }; done
grep -q '^THREEDGS_CAMERA_PREVIEW_PASS$' "$LOG" || ok=0
if [[ "$rc" -eq 0 && "$ok" -eq 1 ]]; then
  echo "HIGH_CAMERA_SMALL_REGION_PASS"
  echo "HIGH_IMAGES=$OUT/preview_frames"
  echo "HIGH_STRIP=$OUT/camera_preview_strip.png"
  echo "SUPERSPLAT_PLY=$CROPPED"
else
  echo "HIGH_CAMERA_SMALL_REGION_FAILED"; [[ "$rc" -ne 0 ]] || rc=102
fi
exit "$rc"
