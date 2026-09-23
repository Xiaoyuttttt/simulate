#!/usr/bin/env bash
set -uo pipefail

ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
ISAAC_PY="/mnt/16T_2/txy/envs/unitree_sim_51/bin/python"
GS_PY="/mnt/16T_2/txy/envs/gsplat_env/bin/python"
PLY="/mnt/16T_2/txy/campus_robot/assets/point_cloud.ply"
GROUND="$ROOT/outputs/control_route_ground/grounded_control_route.csv"
TRAJECTORY="$ROOT/outputs/straight_physics_demo/trajectory_frames.csv"
SUMMARY="$ROOT/outputs/straight_physics_demo/summary.json"
GS_OUT="$ROOT/outputs/straight_physics_3dgs"
CAMERA_PATH="$GS_OUT/camera_path.json"
LOG="$ROOT/logs/17_straight_physics_demo.log"
DISTANCE="${DEMO_DISTANCE_M:-5.0}"
SPEED="${DEMO_SPEED_MPS:-0.30}"
RENDER_PREVIEW="${RENDER_3DGS_PREVIEW:-1}"
RENDER_ALL="${RENDER_ALL_3DGS:-0}"

mkdir -p "$ROOT/logs" "$ROOT/outputs/straight_physics_demo" "$GS_OUT" "$ROOT/stages"
bash "$ROOT/scripts/00_check_isaac51.sh" || exit $?

if [[ ! -s "$GROUND" ]]; then
  echo "Mesh探地路线不存在，先生成地面高度"
  env -u CUDA_VISIBLE_DEVICES PYTHONUNBUFFERED=1 DISPLAY= \
  timeout --foreground --signal=TERM --kill-after=30s 30m \
    "$ISAAC_PY" "$ROOT/scripts/ground_control_route.py" --config "$ROOT/config/placement.json" || exit $?
fi

set +e
env -u CUDA_VISIBLE_DEVICES PYTHONUNBUFFERED=1 DISPLAY= \
timeout --foreground --signal=TERM --kill-after=30s 20m \
  "$ISAAC_PY" "$ROOT/scripts/run_straight_physics_demo.py" \
  --config "$ROOT/config/placement.json" --distance-m "$DISTANCE" --speed-mps "$SPEED" \
  2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}
set -e
if [[ "$rc" -ne 0 ]] || ! grep -q '^STRAIGHT_PHYSICS_DEMO_PASS$' "$LOG" || [[ ! -s "$TRAJECTORY" || ! -s "$SUMMARY" ]]; then
  echo "CAMPUS_STRAIGHT_PHYSICS_DEMO_FAILED"
  [[ "$rc" -ne 0 ]] || rc=122
  exit "$rc"
fi

if [[ "$RENDER_PREVIEW" == "0" && "$RENDER_ALL" == "0" ]]; then
  echo "CAMPUS_STRAIGHT_PHYSICS_DEMO_PASS"
  exit 0
fi

[[ -x "$GS_PY" ]] || { echo "GSPLAT_PYTHON_MISSING=$GS_PY"; exit 123; }
[[ -s "$PLY" ]] || { echo "PLY_MISSING=$PLY"; exit 124; }
"$GS_PY" "$ROOT/scripts/export_3dgs_camera_path.py" \
  --config "$ROOT/config/placement.json" --source-csv "$TRAJECTORY" --output "$CAMERA_PATH" \
  --width 640 --height 360 --target-fps 10 --camera-forward-m 0.45 --camera-height-body-m 1.08 || exit $?

if [[ "$RENDER_PREVIEW" != "0" ]]; then
  CUDA_HOME=/usr/local/cuda-12.4 PATH="/usr/local/cuda-12.4/bin:$PATH" PYTHONUNBUFFERED=1 \
  timeout --foreground --signal=TERM --kill-after=30s 30m \
    "$GS_PY" "$ROOT/scripts/render_3dgs_camera_preview.py" --ply "$PLY" --camera-path "$CAMERA_PATH" \
    --output-dir "$GS_OUT" --max-gaussians "${MAX_GAUSSIANS:-3000000}" \
    --route-crop-margin "${ROUTE_CROP_MARGIN:-40}" --device cuda:0 || exit $?
  [[ -s "$GS_OUT/camera_preview_strip.png" ]] || { echo "STRAIGHT_3DGS_PREVIEW_MISSING"; exit 125; }
fi

if [[ "$RENDER_ALL" != "0" ]]; then
  command -v ffmpeg >/dev/null 2>&1 || { echo "FFMPEG_NOT_FOUND"; exit 126; }
  CUDA_HOME=/usr/local/cuda-12.4 PATH="/usr/local/cuda-12.4/bin:$PATH" PYTHONUNBUFFERED=1 \
  timeout --foreground --signal=TERM --kill-after=60s 4h \
    "$GS_PY" "$ROOT/scripts/render_3dgs_camera_preview.py" --ply "$PLY" --camera-path "$CAMERA_PATH" \
    --output-dir "$GS_OUT" --max-gaussians "${MAX_GAUSSIANS:-3000000}" \
    --route-crop-margin "${ROUTE_CROP_MARGIN:-40}" --device cuda:0 --all-frames --resume || exit $?
  ffmpeg -y -framerate 10 -i "$GS_OUT/background_frames/frame_%04d.png" \
    -c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p -movflags +faststart \
    "$GS_OUT/straight_physics_3dgs.mp4" || exit $?
  [[ -s "$GS_OUT/straight_physics_3dgs.mp4" ]] || { echo "STRAIGHT_3DGS_VIDEO_MISSING"; exit 127; }
fi

echo "TRAJECTORY=$TRAJECTORY"
echo "SUMMARY=$SUMMARY"
echo "CAMERA_PATH=$CAMERA_PATH"
echo "PREVIEW=$GS_OUT/camera_preview_strip.png"
[[ "$RENDER_ALL" == "0" ]] || echo "VIDEO=$GS_OUT/straight_physics_3dgs.mp4"
echo "CAMPUS_STRAIGHT_PHYSICS_DEMO_PASS"
