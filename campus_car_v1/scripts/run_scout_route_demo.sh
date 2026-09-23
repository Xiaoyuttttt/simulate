#!/usr/bin/env bash
set -euo pipefail

ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
ISAAC_PY="/mnt/16T_2/txy/envs/unitree_sim_51/bin/python"
CONFIG="$ROOT/config/placement.json"
CONTROL="$ROOT/config/route_control_points_new.csv"
GROUND_OUT="$ROOT/outputs/scout_v2_route_ground"
GROUND="$GROUND_OUT/grounded_control_route.csv"
STAMP="$(date -u +%Y%m%d_%H%M%S)"
OUT="$ROOT/outputs/scout_v2_route_$STAMP"
LOG="$ROOT/logs/scout_v2_route_$STAMP.log"
FPS="${SCOUT_VIDEO_FPS:-10}"
WIDTH="${SCOUT_VIDEO_WIDTH:-960}"
HEIGHT="${SCOUT_VIDEO_HEIGHT:-540}"

mkdir -p "$ROOT/logs" "$ROOT/outputs" "$ROOT/stages"
[[ -s "$CONTROL" ]] || { echo "ROUTE_CSV_MISSING=$CONTROL"; exit 2; }
[[ -s "$ROOT/assets/scout_v2_isaac/Isaac_models/scout_v2_base.usd" ]] || { echo "SCOUT_USD_MISSING"; exit 3; }
bash "$ROOT/scripts/00_check_isaac51.sh" || exit $?

echo "Grounding new route against campus mesh"
env -u CUDA_VISIBLE_DEVICES PYTHONUNBUFFERED=1 DISPLAY= \
  timeout --foreground --signal=TERM --kill-after=30s 30m "$ISAAC_PY" \
  "$ROOT/scripts/ground_control_route.py" --config "$CONFIG" --input-csv "$CONTROL" --output-dir "$GROUND_OUT" || exit $?

echo "PhysX Scout v2 route-follow and dual-camera capture"
env -u CUDA_VISIBLE_DEVICES PYTHONUNBUFFERED=1 DISPLAY= \
  timeout --foreground --signal=TERM --kill-after=60s 3h "$ISAAC_PY" \
  "$ROOT/scripts/run_scout_route_sim.py" --config "$CONFIG" --grounded-route "$GROUND" \
  --output-dir "$OUT" --fps "$FPS" --width "$WIDTH" --height "$HEIGHT" 2>&1 | tee "$LOG"

[[ -s "$OUT/trajectory_frames.csv" ]] || { echo "SCOUT_TRAJECTORY_MISSING"; exit 4; }
grep -q '^SCOUT_ROUTE_SIM_PASS$' "$LOG" || { echo "SCOUT_ROUTE_SIM_NOT_PASSED"; exit 5; }
command -v ffmpeg >/dev/null 2>&1 || { echo "FFMPEG_NOT_FOUND"; exit 6; }
ffmpeg -y -framerate "$FPS" -i "$OUT/first_person_frames/rgb_%05d.png" \
  -c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p -movflags +faststart \
  "$OUT/scout_first_person.mp4" || exit $?
ffmpeg -y -framerate "$FPS" -i "$OUT/third_person_frames/rgb_%05d.png" \
  -c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p -movflags +faststart \
  "$OUT/scout_third_person.mp4" || exit $?
[[ -s "$OUT/scout_first_person.mp4" && -s "$OUT/scout_third_person.mp4" ]] || { echo "SCOUT_VIDEO_MISSING"; exit 7; }
echo "SCOUT_ROUTE_OUTPUT=$OUT"
echo "FIRST_PERSON_VIDEO=$OUT/scout_first_person.mp4"
echo "THIRD_PERSON_VIDEO=$OUT/scout_third_person.mp4"
echo "TRAJECTORY=$OUT/trajectory_frames.csv"
echo "SCOUT_ROUTE_VIDEO_PASS"
