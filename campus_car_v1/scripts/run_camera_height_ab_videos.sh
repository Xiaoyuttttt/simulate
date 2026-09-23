#!/usr/bin/env bash
set -uo pipefail
ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
GS_PY="/mnt/16T_2/txy/envs/gsplat_env/bin/python"
SOURCE_PLY="/mnt/16T_2/txy/campus_robot/assets/point_cloud.ply"
SOURCE_CSV="$ROOT/outputs/control_route_animation/trajectory_frames.csv"
FPS="${VIDEO_FPS:-12}"; WIDTH="${VIDEO_WIDTH:-1920}"; HEIGHT="${VIDEO_HEIGHT:-1080}"
DURATION="${AB_DURATION:-15}"; MARGIN="${REGION_MARGIN:-80}"; DEVICE="${GS_DEVICE:-cuda:0}"
SCALE_Q="${MAX_SCALE_QUANTILE:-0.995}"
OUT="$ROOT/outputs/camera_height_ab_${DURATION}s_scaleq_${SCALE_Q}"
SHARED="$OUT/shared"
LOW_CAMERA="$OUT/ground_067m/camera_path.json"; HIGH_CAMERA="$OUT/ground_400m/camera_path.json"
CROPPED="$SHARED/campus_route_region_${MARGIN}m.ply"
mkdir -p "$OUT/ground_067m" "$OUT/ground_400m" "$SHARED" "$ROOT/logs"
[[ -x "$GS_PY" && -s "$SOURCE_PLY" && -s "$SOURCE_CSV" ]] || { echo "CAMERA_HEIGHT_AB_INPUT_MISSING"; exit 111; }
command -v ffmpeg >/dev/null 2>&1 || { echo "FFMPEG_NOT_FOUND"; exit 112; }

duration_args=(); if [[ "$DURATION" != "0" && "$DURATION" != "0.0" ]]; then duration_args=(--max-duration-s "$DURATION"); fi
common=(--config "$ROOT/config/placement.json" --source-csv "$SOURCE_CSV" --width "$WIDTH" --height "$HEIGHT" --target-fps "$FPS" --camera-forward-m 0.45 --pitch-down-deg 0 "${duration_args[@]}")
echo "[1/5] 导出严格对照相机，唯一变量为光心相对局部地面的高度"
"$GS_PY" "$ROOT/scripts/export_3dgs_camera_path.py" "${common[@]}" --camera-height-ground-m 0.67 --output "$LOW_CAMERA" || exit $?
"$GS_PY" "$ROOT/scripts/export_3dgs_camera_path.py" "${common[@]}" --camera-height-ground-m 4.00 --output "$HIGH_CAMERA" || exit $?

echo "[2/5] 从完整PLY裁剪双方共用的路线周围${MARGIN}m区域"
"$GS_PY" "$ROOT/scripts/crop_3dgs_route_region.py" --input "$SOURCE_PLY" --camera-path "$LOW_CAMERA" --output "$CROPPED" --margin "$MARGIN" || exit $?

render_one() {
  local label="$1" camera="$2" folder="$OUT/$1" log="$ROOT/logs/16_camera_height_ab_${1}_scaleq_${SCALE_Q}.log"
  echo "渲染 $label"
  CUDA_HOME=/usr/local/cuda-12.4 PATH="/usr/local/cuda-12.4/bin:$PATH" PYTHONUNBUFFERED=1 \
  timeout --foreground --signal=TERM --kill-after=60s 12h \
   "$GS_PY" "$ROOT/scripts/render_3dgs_camera_preview.py" --ply "$CROPPED" --camera-path "$camera" \
   --output-dir "$folder" --max-gaussians 0 --route-crop-margin 0 --preserve-all-valid --max-scale-quantile "$SCALE_Q" --rasterize-mode antialiased --device "$DEVICE" --all-frames 2>&1 | tee "$log"
  local rc=${PIPESTATUS[0]}; [[ "$rc" -eq 0 ]] || return "$rc"
  grep -q '^THREEDGS_CAMERA_FULL_PASS$' "$log" || return 113
  local expected count; expected=$("$GS_PY" -c 'import json,sys; print(len(json.load(open(sys.argv[1]))))' "$camera")
  count=$(find "$folder/background_frames" -maxdepth 1 -type f -name 'frame_*.png' | wc -l)
  [[ "$count" -eq "$expected" ]] || { echo "FRAME_COUNT_MISMATCH=$label:$count/$expected"; return 114; }
  ffmpeg -y -framerate "$FPS" -i "$folder/background_frames/frame_%04d.png" -c:v libx264 -preset slow -crf 14 -pix_fmt yuv420p -movflags +faststart "$folder/${label}.mp4" || return $?
}

echo "[3/5] A组：Gemini 335光心严格离局部地面0.67m"
render_one ground_067m "$LOW_CAMERA" || exit $?
echo "[4/5] B组：光心严格离局部地面4.00m"
render_one ground_400m "$HIGH_CAMERA" || exit $?

echo "[5/5] 完成"
echo "AB_CONSTANTS resolution=${WIDTH}x${HEIGHT} fps=$FPS duration=${DURATION}s pitch=0deg forward=0.45m crop=${MARGIN}m rasterize=antialiased sampling=none opacity_filter=none max_scale_quantile=$SCALE_Q crf=14"
echo "VIDEO_A=$OUT/ground_067m/ground_067m.mp4"
echo "VIDEO_B=$OUT/ground_400m/ground_400m.mp4"
echo "SHARED_PLY=$CROPPED"
echo "CAMERA_HEIGHT_AB_PASS"
