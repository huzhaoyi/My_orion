#!/usr/bin/env bash
# 从 rosbag2 抽取插孔段 (s_t, a_t, s_{t+1})。
#
# 用法:
#   ./extract_insert_trajectories.sh bags/ep_target_001
#   ./extract_insert_trajectories.sh --jobs-jsonl jobs.jsonl --merge-out analysis/insert_extract/all.npz
#
# 前置: source ROS2 + 工作空间（holoocean_interfaces、mtc_msgs、holoocean_bridge）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "${ROS_DISTRO:-}" ]
then
    echo "[extract_insert_trajectories] 请先 source ROS2 与工作空间，例如:"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  source <ws>/install/setup.bash"
    exit 1
fi

exec python3 "${SCRIPT_DIR}/extract_insert_trajectories.py" "$@"
