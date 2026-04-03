#!/usr/bin/env bash
# 一键分析 bags 中的 rosbag（无需再手动敲 python 路径）。
# 默认: 使用 bags/ 下「最近修改」的子目录；也可传入子目录名或绝对路径。
# 依赖: 当前 shell 已 source ROS2 与含 holoocean_interfaces 的 install；可选 pip install -r requirements.txt
# 输出: analysis/<bag 目录名>/（png + csv）
# 可选: 环境变量 ARM_BAG_OPEN=1 时尝试用图像查看器打开 png（需 DISPLAY）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAGS_ROOT="${SCRIPT_DIR}/bags"
ANALYSIS_ROOT="${SCRIPT_DIR}/analysis"

if [ -z "${ROS_DISTRO:-}" ]
then
    echo "[analyze_arm_bag] 请先在本终端执行:"
    echo "  source /opt/ros/<distro>/setup.bash"
    echo "  source <你的工作空间>/install/setup.bash"
    exit 1
fi

if [ -n "${1:-}" ]
then
    if [[ "${1}" = /* ]]
    then
        BAG_DIR="${1}"
    else
        BAG_DIR="${BAGS_ROOT}/${1}"
    fi
else
    # bags/ 下最近修改的子目录（按目录 mtime）
    BAG_DIR=""
    latest_line="$(ls -td "${BAGS_ROOT}"/*/ 2>/dev/null | head -1 || true)"
    if [ -n "${latest_line}" ]
    then
        BAG_DIR="${latest_line%/}"
    fi
fi

if [ -z "${BAG_DIR}" ] || [ ! -d "${BAG_DIR}" ]
then
    echo "[analyze_arm_bag] 未找到 bag 目录。请先运行 ./record_arm_chain.sh 或将 bag 路径作为参数传入。"
    exit 1
fi

if ! ls "${BAG_DIR}"/*.db3 >/dev/null 2>&1
then
    echo "[analyze_arm_bag] 目录中无 .db3: ${BAG_DIR}"
    exit 1
fi

BAG_NAME="$(basename "${BAG_DIR}")"
OUT_DIR="${ANALYSIS_ROOT}/${BAG_NAME}"
mkdir -p "${OUT_DIR}"

echo "[analyze_arm_bag] 输入: ${BAG_DIR}"
echo "[analyze_arm_bag] 输出: ${OUT_DIR}"

python3 "${SCRIPT_DIR}/plot_arm_bag.py" "${BAG_DIR}" --out "${OUT_DIR}"

echo "[analyze_arm_bag] 完成。"

if [ "${ARM_BAG_OPEN:-0}" = "1" ] && [ -n "${DISPLAY:-}" ] && command -v xdg-open >/dev/null 2>&1
then
    xdg-open "${OUT_DIR}/arm_comparison.png" >/dev/null 2>&1 || true
fi
