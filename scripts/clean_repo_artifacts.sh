#!/usr/bin/env bash
# 清理 git 仓库根目录误生成的 colcon 产物，以及 orion_* 重命名后的本地空壳/__pycache__。
# 正常编译请在 sealien_ws 根目录：colcon build && source install/setup.bash
set -euo pipefail

_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_repo_root="$(cd "${_script_dir}/.." && pwd)"

cd "${_repo_root}"

echo "清理目录: ${_repo_root}"

rm -rf build install log .pytest_cache

# 重命名后残留的旧包目录（通常只剩 __pycache__）
for legacy in src/orion_bringup src/orion_description src/orion_moveit_config \
    src/orion_mtc_msgs src/orion_mtc src/orion_holoocean_bridge src/orion_joy_arm_bridge
do
    if [ -d "${legacy}" ]
    then
        echo "  删除残留: ${legacy}"
        rm -rf "${legacy}"
    fi
done

# 递归删除 __pycache__（git 已忽略）
find "${_repo_root}" -type d -name '__pycache__' -prune -exec rm -rf {} + 2>/dev/null || true

echo "完成。请在 sealien_ws 根目录编译："
echo "  cd <sealien_ws> && colcon build --symlink-install && source install/setup.bash"
