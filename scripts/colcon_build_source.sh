#!/usr/bin/env bash
# 在 colcon 工作空间根目录执行 build，并自动 source install/setup.bash，
# 避免出现「已编译包但 ros2 launch 找不到包」（当前 shell 未 overlay 到 install）。
#
# 默认假定本仓库路径为: <工作空间>/src/Sealien_CtrlPilot_Payload_Orion
# 也可先 export COLCON_WS=/path/to/sealien_ws 再运行本脚本。
set -euo pipefail

_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_repo_root="$(cd "${_script_dir}/.." && pwd)"
_ws="${COLCON_WS:-}"
if [ -z "${_ws}" ]
then
    _ws="$(cd "${_repo_root}/../.." && pwd)"
fi

if [ ! -d "${_ws}/install" ]
then
    echo "错误: 未找到 ${_ws}/install ，请确认 COLCON_WS 或仓库在 <ws>/src/<本仓库> 下。" >&2
    exit 1
fi

if [ -f /opt/ros/humble/setup.bash ]
then
    # shellcheck source=/dev/null
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/${ROS_DISTRO:-humble}/setup.bash ]
then
    # shellcheck source=/dev/null
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
    echo "错误: 未找到 /opt/ros 下 setup.bash，请先安装 ROS 2。" >&2
    exit 1
fi

cd "${_ws}"
colcon build "$@"
# shellcheck source=/dev/null
source "${_ws}/install/setup.bash"
echo "已 source: ${_ws}/install/setup.bash （当前终端已加载工作空间 overlay）"
