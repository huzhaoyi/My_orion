#!/usr/bin/env bash
# 为本仓库各 ROS 2 包安装可通过 rosdep / apt 解析的系统依赖（Ubuntu + 官方 ROS apt 源）。
# 不包含第三方工作空间包：holoocean_interfaces、sealien_ctrlpilot_msgmanagement（见 docs/DEPENDENCIES.md）。
#
# 用法:
#   ./scripts/install_ros_dependencies.sh              # 默认 rosdep（推荐）
#   ROS_DISTRO=jazzy ./scripts/install_ros_dependencies.sh
#   ./scripts/install_ros_dependencies.sh --apt-explicit   # 不用 rosdep，按包名列表 apt install（网络差时可试）
#
set -euo pipefail

_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_repo_root="$(cd "${_script_dir}/.." && pwd)"
_ros_distro="${ROS_DISTRO:-humble}"
_use_rosdep=1

for _arg in "$@"
do
    if [ "${_arg}" = "--apt-explicit" ]
    then
        _use_rosdep=0
    elif [ "${_arg}" = "-h" ] || [ "${_arg}" = "--help" ]
    then
        echo "用法: $(basename "${BASH_SOURCE[0]}") [--apt-explicit] [--help]"
        echo "  默认: rosdep install（跳过 holoocean_interfaces、sealien_ctrlpilot_msgmanagement）"
        echo "  --apt-explicit: 不使用 rosdep，按固定 ros-\${ROS_DISTRO}-* 列表 apt install"
        echo "  环境变量: ROS_DISTRO（默认 humble）"
        exit 0
    fi
done

if [ ! -f "/opt/ros/${_ros_distro}/setup.bash" ]
then
    echo "错误: 未找到 /opt/ros/${_ros_distro}/setup.bash。请先安装 ROS 2 ${_ros_distro} 并配置 apt 源。" >&2
    echo "文档: https://docs.ros.org/en/${_ros_distro}/Installation/Ubuntu-Install-Debians.html" >&2
    exit 1
fi

if [ ! -d "${_repo_root}/src" ]
then
    echo "错误: 未找到 ${_repo_root}/src（请在仓库根目录保持目录结构）。" >&2
    exit 1
fi

echo "==> 安装编译常用工具与 rosdep（若已安装会跳过）"
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    python3-colcon-common-extensions \
    python3-rosdep

if [ "${_use_rosdep}" -eq 1 ]
then
    echo "==> rosdep（首次使用需 init / update，可能需要网络）"
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]
    then
        sudo rosdep init || true
    fi
    rosdep update

    # shellcheck source=/dev/null
    source "/opt/ros/${_ros_distro}/setup.bash"

    echo "==> rosdep install --from-paths ${_repo_root}/src （跳过外部工作空间键）"
    set +e
    rosdep install \
        --from-paths "${_repo_root}/src" \
        --ignore-src \
        --rosdistro "${_ros_distro}" \
        -y \
        -r \
        --skip-keys="holoocean_interfaces sealien_ctrlpilot_msgmanagement"
    _rosdep_status=$?
    set -e
    if [ "${_rosdep_status}" -ne 0 ]
    then
        echo "" >&2
        echo "rosdep 未完全成功（常见原因：网络、rosdistro 键缺失）。可改用:" >&2
        echo "  ${_script_dir}/install_ros_dependencies.sh --apt-explicit" >&2
        exit "${_rosdep_status}"
    fi
else
    echo "==> 按固定 apt 包列表安装（${_ros_distro}）"
    _p="ros-${_ros_distro}"
    sudo apt-get install -y \
        "${_p}-rclcpp" \
        "${_p}-rclpy" \
        "${_p}-ament-cmake" \
        "${_p}-moveit-task-constructor-core" \
        "${_p}-moveit-task-constructor-msgs" \
        "${_p}-moveit-ros-planning-interface" \
        "${_p}-moveit-core" \
        "${_p}-moveit-ros-planning" \
        "${_p}-moveit-ros-move-group" \
        "${_p}-moveit-planners-ompl" \
        "${_p}-pilz-industrial-motion-planner" \
        "${_p}-moveit-ros-visualization" \
        "${_p}-moveit-simple-controller-manager" \
        "${_p}-geometry-msgs" \
        "${_p}-shape-msgs" \
        "${_p}-moveit-msgs" \
        "${_p}-control-msgs" \
        "${_p}-rclcpp-action" \
        "${_p}-trajectory-msgs" \
        "${_p}-std-srvs" \
        "${_p}-builtin-interfaces" \
        "${_p}-sensor-msgs" \
        "${_p}-nav-msgs" \
        "${_p}-std-msgs" \
        "${_p}-tf2" \
        "${_p}-tf2-ros" \
        "${_p}-tf2-geometry-msgs" \
        "${_p}-tf2-eigen" \
        "${_p}-rosidl-default-generators" \
        "${_p}-rosidl-default-runtime" \
        "${_p}-action-msgs" \
        "${_p}-rviz2" \
        "${_p}-joint-state-publisher" \
        "${_p}-joint-state-publisher-gui" \
        "${_p}-robot-state-publisher" \
        "${_p}-xacro" \
        "${_p}-ament-index-python" \
        "${_p}-ros2launch" \
        "${_p}-rosbridge-server" \
        "${_p}-rosapi"
fi

echo ""
echo "完成。请另行准备（本脚本不安装）："
echo "  - holoocean_interfaces（HoloOcean ROS 工作空间，环境变量 HOLOOCEAN_ROS_INSTALL 或 source 其 install）"
echo "  - sealien_ctrlpilot_msgmanagement（同一 colcon 工作空间或先 source 其 overlay）"
echo "详见: ${_repo_root}/docs/DEPENDENCIES.md"
