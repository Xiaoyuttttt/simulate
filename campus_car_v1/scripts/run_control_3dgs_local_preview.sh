#!/usr/bin/env bash
set -uo pipefail
ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
GS_PY="/mnt/16T_2/txy/envs/gsplat_env/bin/python"
PLY="/mnt/16T_2/txy/campus_robot/assets/point_cloud.ply"
SOURCE="$ROOT/outputs/control_route_animation/trajectory_frames.csv"
OUT="$ROOT/outputs/control_3dgs_local_preview"
CAMERA="$OUT/camera_path.json"
LOG="$ROOT/logs/13_control_3dgs_local_preview.log"
WIDTH="${VIDEO_WIDTH:-1280}"; HEIGHT="${VIDEO_HEIGHT:-720}"; MARGIN="${ROUTE_CROP_MARGIN:-120}"
mkdir -p "$OUT" "$ROOT/logs"
[[ -x "$GS_PY" && -s "$PLY" && -s "$SOURCE" ]] || { echo "LOCAL_PREVIEW_INPUT_MISSING"; exit 91; }
"$GS_PY" "$ROOT/scripts/export_3dgs_camera_path.py" --config "$ROOT/config/placement.json" \
 --source-csv "$SOURCE" --output "$CAMERA" --width "$WIDTH" --height "$HEIGHT" --target-fps 12 || exit $?
set +e
CUDA_HOME=/usr/local/cuda-12.4 PATH="/usr/local/cuda-12.4/bin:$PATH" PYTHONUNBUFFERED=1 \
timeout --foreground --signal=TERM --kill-after=60s 2h \
 "$GS_PY" "$ROOT/scripts/render_3dgs_camera_preview.py" --ply "$PLY" --camera-path "$CAMERA" \
 --output-dir "$OUT" --max-gaussians 0 --route-crop-margin "$MARGIN" --device cuda:0 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}; set -e
required=("$OUT/camera_preview_strip.png" "$OUT/summary.json")
ok=1; for p in "${required[@]}"; do [[ -s "$p" ]] || { echo "MISSING_OUTPUT=$p"; ok=0; }; done
grep -q '^THREEDGS_CAMERA_PREVIEW_PASS$' "$LOG" || ok=0
if [[ "$rc" -eq 0 && "$ok" -eq 1 ]]; then echo "CAMPUS_CONTROL_3DGS_LOCAL_PREVIEW_PASS"; else echo "CAMPUS_CONTROL_3DGS_LOCAL_PREVIEW_FAILED"; [[ "$rc" -ne 0 ]] || rc=92; fi
exit "$rc"
