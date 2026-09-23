#!/usr/bin/env bash
set -uo pipefail
ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
PYTHON="/mnt/16T_2/txy/envs/unitree_sim_51/bin/python"
OUT="$ROOT/outputs/first_person"
LOG="$ROOT/logs/04_first_person.log"
VIDEO="$OUT/campus_car_first_person.mp4"
mkdir -p "$ROOT/logs" "$OUT"
bash "$ROOT/scripts/00_check_isaac51.sh" || exit $?
command -v ffmpeg >/dev/null 2>&1 || { echo "FFMPEG_NOT_FOUND"; exit 51; }
set +e
env -u CUDA_VISIBLE_DEVICES PYTHONUNBUFFERED=1 DISPLAY= timeout --foreground --signal=TERM --kill-after=30s 45m \
  "$PYTHON" "$ROOT/scripts/render_first_person.py" --config "$ROOT/config/placement.json" --width 960 --height 540 \
  2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}
set -e
if [[ "$rc" -ne 0 ]] || ! grep -q '^FIRST_PERSON_FRAMES_PASS$' "$LOG"; then
  echo "FIRST_PERSON_RENDER_FAILED"; exit "${rc:-52}"
fi
count=$(find "$OUT/frames" -maxdepth 1 -type f -name 'frame_*.png' | wc -l)
[[ "$count" -eq 271 ]] || { echo "FRAME_COUNT_MISMATCH=$count expected=271"; exit 53; }
ffmpeg -y -framerate 15 -i "$OUT/frames/frame_%04d.png" -c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p -movflags +faststart "$VIDEO"
[[ -s "$VIDEO" ]] || { echo "VIDEO_MISSING=$VIDEO"; exit 54; }
echo "FIRST_PERSON_VIDEO=$VIDEO"
echo "CAMPUS_CAR_FIRST_PERSON_PASS"
