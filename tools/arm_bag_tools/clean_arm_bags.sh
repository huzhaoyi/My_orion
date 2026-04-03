#!/usr/bin/env bash
# 清空 tools/arm_bag_tools/bags/ 下默认录制的 bag 目录，保留 .gitkeep。
# 用法:
#   ./clean_arm_bags.sh          # 交互确认后删除
#   ./clean_arm_bags.sh -y       # 不询问直接删除

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAGS_DIR="${SCRIPT_DIR}/bags"

if [ ! -d "${BAGS_DIR}" ]
then
    echo "[clean_arm_bags] 无目录: ${BAGS_DIR}"
    exit 0
fi

count=0
while IFS= read -r -d '' p
do
    count=$((count + 1))
done < <(find "${BAGS_DIR}" -mindepth 1 -maxdepth 1 ! -name '.gitkeep' -print0 2>/dev/null)

if [ "${count}" -eq 0 ]
then
    echo "[clean_arm_bags] ${BAGS_DIR} 下已无 bag 子目录"
    exit 0
fi

if [ "${1:-}" != "-y" ]
then
    echo "[clean_arm_bags] 将删除 ${BAGS_DIR} 下共 ${count} 项（保留 .gitkeep）。确认？[y/N]"
    read -r ans
    if [ "${ans}" != "y" ] && [ "${ans}" != "Y" ]
    then
        echo "[clean_arm_bags] 已取消"
        exit 0
    fi
fi

find "${BAGS_DIR}" -mindepth 1 -maxdepth 1 ! -name '.gitkeep' -exec rm -rf {} +
echo "[clean_arm_bags] 已清空（保留 .gitkeep）"
