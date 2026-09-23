#!/usr/bin/env bash
set -uo pipefail
ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
GS_PY="/mnt/16T_2/txy/envs/gsplat_env/bin/python"
PLY="/mnt/16T_2/txy/campus_robot/assets/point_cloud.ply"
OUT="$ROOT/outputs/3dgs_camera"
LOG="$ROOT/logs/05_3dgs_camera_preview.log"
mkdir -p "$OUT" "$ROOT/logs"
[[ -x "$GS_PY" ]] || { echo "GSPLAT_PYTHON_MISSING=$GS_PY"; exit 61; }
[[ -s "$PLY" ]] || { echo "PLY_MISSING=$PLY"; exit 62; }
"$GS_PY" -c 'import torch,gsplat,plyfile,PIL; assert torch.cuda.is_available(); print("GSPLAT_ENV_PASS",torch.__version__)' || exit $?
"$GS_PY" "$ROOT/scripts/export_3dgs_camera_path.py" --config "$ROOT/config/placement.json" || exit $?
set +e
CUDA_HOME=/usr/local/cuda-12.4 PATH="/usr/local/cuda-12.4/bin:$PATH" PYTHONUNBUFFERED=1 timeout --foreground --kill-after=30s 30m \
 "$GS_PY" "$ROOT/scripts/render_3dgs_camera_preview.py" --ply "$PLY" --camera-path "$OUT/camera_path.json" \
 --output-dir "$OUT" --max-gaussians "${MAX_GAUSSIANS:-5000000}" --device cuda:0 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}; set -e
[[ "$rc" -eq 0 ]] && grep -q '^THREEDGS_CAMERA_PREVIEW_PASS$' "$LOG" && [[ -s "$OUT/camera_preview_strip.png" ]] \
 && echo "CAMPUS_3DGS_CAMERA_PREVIEW_PASS" && exit 0
echo "CAMPUS_3DGS_CAMERA_PREVIEW_FAILED"; exit "${rc:-63}"
