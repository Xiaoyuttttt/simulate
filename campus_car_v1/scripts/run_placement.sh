#!/usr/bin/env bash
set -uo pipefail

PROJECT_ROOT="/mnt/16T_2/txy/campus_robot/project/campus_car_v1"
PYTHON_BIN="/mnt/16T_2/txy/envs/unitree_sim_51/bin/python"
CONFIG="$PROJECT_ROOT/config/placement.json"
LOG_DIR="$PROJECT_ROOT/logs"
RUN_LOG="$LOG_DIR/01_place_car_p1.log"

mkdir -p "$LOG_DIR" "$PROJECT_ROOT/outputs/placement_p1" "$PROJECT_ROOT/stages"

echo "[1/3] 验证txy自己的 Isaac Sim 5.1"
set +e
bash "$PROJECT_ROOT/scripts/00_check_isaac51.sh"
preflight_rc=$?
set -e
[[ "$preflight_rc" -eq 0 ]] || exit "$preflight_rc"

echo "[2/3] 检查输入资产"
"$PYTHON_BIN" - "$CONFIG" <<'PY'
import json, sys
from pathlib import Path
cfg = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
missing = []
if not Path(cfg["waypoints_csv"]).is_file():
    missing.append(cfg["waypoints_csv"])
if not Path(cfg["campus_usd"]).is_file() and not Path(cfg["campus_obj"]).is_file():
    missing.extend([cfg["campus_usd"], cfg["campus_obj"]])
if missing:
    print("MISSING_INPUTS")
    print("\n".join(missing))
    raise SystemExit(5)
print("INPUT_ASSETS_PASS")
PY
asset_rc=$?
[[ "$asset_rc" -eq 0 ]] || exit "$asset_rc"

echo "[3/3] Isaac Sim 5.1 Headless放车、探地并输出三张检查图（最长20分钟）"
set +e
env -u CUDA_VISIBLE_DEVICES \
PYTHONUNBUFFERED=1 DISPLAY= \
timeout --foreground --signal=TERM --kill-after=30s 20m \
    "$PYTHON_BIN" "$PROJECT_ROOT/scripts/place_car_p1.py" --config "$CONFIG" \
    2>&1 | tee "$RUN_LOG"
python_rc=${PIPESTATUS[0]}
set -e

echo "PYTHON_EXIT_CODE=$python_rc"
echo "PLACEMENT_JSON=$PROJECT_ROOT/outputs/placement_p1/placement.json"
echo "CHECK_IMAGES=$PROJECT_ROOT/outputs/placement_p1"
echo "STAGE=$PROJECT_ROOT/stages/campus_car_p1.usd"
echo "LOG=$RUN_LOG"

required_outputs=(
    "$PROJECT_ROOT/outputs/placement_p1/placement.json"
    "$PROJECT_ROOT/outputs/placement_p1/ground_probe_diagnostics.json"
    "$PROJECT_ROOT/outputs/placement_p1/close.png"
    "$PROJECT_ROOT/outputs/placement_p1/side.png"
    "$PROJECT_ROOT/outputs/placement_p1/top.png"
    "$PROJECT_ROOT/stages/campus_car_p1.usd"
)
artifacts_ok=1
for output in "${required_outputs[@]}"; do
    if [[ ! -s "$output" ]]; then
        echo "MISSING_OUTPUT=$output"
        artifacts_ok=0
    fi
done
marker_ok=0
if grep -q '^CAMPUS_CAR_P1_PLACEMENT_PASS$' "$RUN_LOG"; then
    marker_ok=1
else
    echo "MISSING_SUCCESS_MARKER=CAMPUS_CAR_P1_PLACEMENT_PASS"
fi

if [[ "$python_rc" -eq 124 ]]; then
    echo "PLACEMENT_TIMEOUT"
elif [[ "$python_rc" -eq 0 && "$artifacts_ok" -eq 1 && "$marker_ok" -eq 1 ]]; then
    echo "CAMPUS_CAR_P1_PASS"
else
    echo "CAMPUS_CAR_P1_FAILED"
    [[ "$python_rc" -ne 0 ]] || python_rc=21
fi
exit "$python_rc"
