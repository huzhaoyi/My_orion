#!/usr/bin/env bash
# 清空 arm_bag_tools 下录制与分析产物：
#   bags/*       （保留 bags/.gitkeep）
#   analysis/*   （保留 analysis/.gitkeep）
# 用法:
#   ./clean_arm_bags.sh          # 交互确认后删除
#   ./clean_arm_bags.sh -y       # 不询问直接删除

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAGS_DIR="${SCRIPT_DIR}/bags"
ANALYSIS_DIR="${SCRIPT_DIR}/analysis"

paths=()
if [ -d "${BAGS_DIR}" ]
then
    paths+=("${BAGS_DIR}")
fi
if [ -d "${ANALYSIS_DIR}" ]
then
    paths+=("${ANALYSIS_DIR}")
fi

if [ ${#paths[@]} -eq 0 ]
then
    echo "[clean_arm_bags] 无 bags/ 与 analysis/ 目录"
    exit 0
fi

count=0
while IFS= read -r -d '' p
do
    count=$((count + 1))
done < <(find "${paths[@]}" -mindepth 1 -maxdepth 1 ! -name '.gitkeep' -print0 2>/dev/null)

if [ "${count}" -eq 0 ]
then
    echo "[clean_arm_bags] bags/ 与 analysis/ 下已无子项（仅 .gitkeep）"
    exit 0
fi

if [ "${1:-}" != "-y" ]
then
    echo "[clean_arm_bags] 将删除以下根目录内共 ${count} 项（保留各目录下 .gitkeep）："
    echo "  - ${BAGS_DIR}"
    echo "  - ${ANALYSIS_DIR}"
    echo "[clean_arm_bags] 确认？[y/N]"
    read -r ans
    if [ "${ans}" != "y" ] && [ "${ans}" != "Y" ]
    then
        echo "[clean_arm_bags] 已取消"
        exit 0
    fi
fi

find "${paths[@]}" -mindepth 1 -maxdepth 1 ! -name '.gitkeep' -exec rm -rf {} +
echo "[clean_arm_bags] 已清空 bags/ 与 analysis/（保留 .gitkeep）"
