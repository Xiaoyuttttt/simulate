#!/usr/bin/env bash
set -uo pipefail
ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
GS_PY="/mnt/16T_2/txy/envs/gsplat_env/bin/python"
PLY="/mnt/16T_2/txy/campus_robot/assets/point_cloud.ply"
CAMERA_OUT="$ROOT/outputs/3dgs_camera"
OUT="$ROOT/outputs/3dgs_locked_route"
FRAMES="$OUT/background_frames"
VIDEO="$OUT/campus_3dgs_first_person.mp4"
LOG="$ROOT/logs/06_3dgs_full_video.log"
mkdir -p "$OUT" "$ROOT/logs"
[[ -x "$GS_PY" ]] || { echo "GSPLAT_PYTHON_MISSING=$GS_PY"; exit 71; }
[[ -s "$PLY" ]] || { echo "PLY_MISSING=$PLY"; exit 72; }
"$GS_PY" "$ROOT/scripts/export_3dgs_camera_path.py" --config "$ROOT/config/placement.json" || exit $?
command -v ffmpeg >/dev/null 2>&1 || { echo "FFMPEG_NOT_FOUND"; exit 73; }
set +e
CUDA_HOME=/usr/local/cuda-12.4 PATH="/usr/local/cuda-12.4/bin:$PATH" PYTHONUNBUFFERED=1 timeout --foreground --kill-after=60s 4h \
 "$GS_PY" "$ROOT/scripts/render_3dgs_camera_preview.py" --ply "$PLY" --camera-path "$CAMERA_OUT/camera_path.json" \
 --output-dir "$OUT" --max-gaussians "${MAX_GAUSSIANS:-5000000}" --device cuda:0 --all-frames --resume \
 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}; set -e
if [[ "$rc" -ne 0 ]] || ! grep -q '^THREEDGS_CAMERA_FULL_PASS$' "$LOG"; then echo "THREEDGS_FULL_RENDER_FAILED"; exit "${rc:-74}"; fi
count=$(find "$FRAMES" -maxdepth 1 -type f -name 'frame_*.png' | wc -l)
[[ "$count" -eq 271 ]] || { echo "FRAME_COUNT_MISMATCH=$count expected=271"; exit 75; }
ffmpeg -y -framerate 15 -i "$FRAMES/frame_%04d.png" -c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p -movflags +faststart "$VIDEO"
[[ -s "$VIDEO" ]] || { echo "VIDEO_MISSING=$VIDEO"; exit 76; }
echo "THREEDGS_FIRST_PERSON_VIDEO=$VIDEO"
echo "CAMPUS_3DGS_FULL_VIDEO_PASS"
