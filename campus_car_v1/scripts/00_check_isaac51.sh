#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="/mnt/16T_2/txy/envs/unitree_sim_51/bin/python"

if [[ ! -x "$PYTHON_BIN" ]]; then
    echo "ERROR: 找不到txy自己的Isaac Sim Python：$PYTHON_BIN" >&2
    exit 2
fi

info="$($PYTHON_BIN - <<'PY'
import importlib.metadata as metadata
import sys

try:
    version = metadata.version("isaacsim")
except metadata.PackageNotFoundError:
    raise SystemExit("ISAACSIM_PACKAGE_MISSING")

print(f"PYTHON={sys.executable}")
print(f"PYTHON_VERSION={sys.version.split()[0]}")
print(f"ISAAC_SIM_VERSION={version}")
if not version.startswith("5.1."):
    raise SystemExit(5)
PY
)" || rc=$?

rc="${rc:-0}"
printf '%s\n' "$info"

if [[ "$rc" -eq 5 ]]; then
    echo "ERROR: 本流水线要求Isaac Sim 5.1.x，不会使用其他用户环境" >&2
    exit 5
elif [[ "$rc" -ne 0 ]]; then
    echo "ERROR: 无法读取Isaac Sim版本，exit=$rc" >&2
    exit "$rc"
fi

echo "ISAAC51_ENVIRONMENT_PASS"
echo "BOUNDARY=headless_only,no_nurec,no_vnc,single_gpu"
