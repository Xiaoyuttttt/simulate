#!/usr/bin/env bash
set -uo pipefail
ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
GS_PY="/mnt/16T_2/txy/envs/gsplat_env/bin/python"
PLY="/mnt/16T_2/txy/campus_robot/assets/point_cloud.ply"
SOURCE="$ROOT/outputs/control_route_animation/trajectory_frames.csv"
MARGIN="${ROUTE_CROP_MARGIN:-0}"
if [[ "$MARGIN" == "0" || "$MARGIN" == "0.0" ]]; then OUT="$ROOT/outputs/control_3dgs_hq"; else OUT="$ROOT/outputs/control_3dgs_local_hq"; fi
CAMERA="$OUT/camera_path.json"
FRAMES="$OUT/background_frames"
VIDEO="$OUT/campus_control_route_3dgs_hq.mp4"
LOG="$ROOT/logs/12_control_3dgs_hq.log"
FPS="${VIDEO_FPS:-12}"
WIDTH="${VIDEO_WIDTH:-1280}"
HEIGHT="${VIDEO_HEIGHT:-720}"
GAUSSIANS="${MAX_GAUSSIANS:-10000000}"
DEVICE="${GS_DEVICE:-cuda:0}"
mkdir -p "$OUT" "$ROOT/logs"
[[ -x "$GS_PY" ]] || { echo "GSPLAT_PYTHON_MISSING=$GS_PY"; exit 81; }
[[ -s "$PLY" ]] || { echo "PLY_MISSING=$PLY"; exit 82; }
[[ -s "$SOURCE" ]] || { echo "CONTROL_TRAJECTORY_MISSING=$SOURCE"; exit 83; }
command -v ffmpeg >/dev/null 2>&1 || { echo "FFMPEG_NOT_FOUND"; exit 84; }

"$GS_PY" "$ROOT/scripts/export_3dgs_camera_path.py" --config "$ROOT/config/placement.json" \
 --source-csv "$SOURCE" --output "$CAMERA" --width "$WIDTH" --height "$HEIGHT" --target-fps "$FPS" || exit $?
expected=$("$GS_PY" -c 'import json,sys; print(len(json.load(open(sys.argv[1]))))' "$CAMERA")
echo "HQ_CONFIG gaussians=$GAUSSIANS resolution=${WIDTH}x${HEIGHT} fps=$FPS frames=$expected device=$DEVICE"
nvidia-smi --query-gpu=index,name,memory.total,memory.free --format=csv,noheader || true

set +e
CUDA_HOME=/usr/local/cuda-12.4 PATH="/usr/local/cuda-12.4/bin:$PATH" PYTHONUNBUFFERED=1 \
timeout --foreground --signal=TERM --kill-after=60s 12h \
 "$GS_PY" "$ROOT/scripts/render_3dgs_camera_preview.py" --ply "$PLY" --camera-path "$CAMERA" \
 --output-dir "$OUT" --max-gaussians "$GAUSSIANS" --route-crop-margin "$MARGIN" --device "$DEVICE" --all-frames --resume 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}; set -e
if [[ "$rc" -ne 0 ]] || ! grep -q '^THREEDGS_CAMERA_FULL_PASS$' "$LOG"; then
 echo "CONTROL_3DGS_HQ_RENDER_FAILED"; [[ "$rc" -ne 0 ]] || rc=85; exit "$rc"
fi
count=$(find "$FRAMES" -maxdepth 1 -type f -name 'frame_*.png' | wc -l)
[[ "$count" -eq "$expected" ]] || { echo "FRAME_COUNT_MISMATCH=$count expected=$expected"; exit 86; }
ffmpeg -y -framerate "$FPS" -i "$FRAMES/frame_%04d.png" -c:v libx264 -preset medium -crf 16 -pix_fmt yuv420p -movflags +faststart "$VIDEO"
[[ -s "$VIDEO" ]] || { echo "VIDEO_MISSING=$VIDEO"; exit 87; }
echo "CONTROL_3DGS_HQ_VIDEO=$VIDEO"
echo "CAMPUS_CONTROL_3DGS_HQ_PASS"
