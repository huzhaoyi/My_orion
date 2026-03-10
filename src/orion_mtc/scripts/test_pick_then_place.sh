#!/usr/bin/env bash
# 按顺序测试：仅抓取 -> 仅放置，并检查状态机与持物逻辑。
# 前置：终端 1 已运行 ros2 launch orion_mtc pick_place_holoocean.launch.py
# 本脚本在终端 2 执行，且已 source install/setup.bash

set -e

echo "=============================================="
echo "  测试顺序：仅抓取 -> 仅放置（状态与持物检查）"
echo "=============================================="

# 解析 get_robot_state 返回的 mode（ros2 service call 可能打印为 "mode: IDLE" 或 "mode='IDLE'"）
extract_mode() {
  echo "$1" | sed -n \
    -e "s/.*mode: *['\\\"]\\{0,1\\}\\([^'\\\" ]\\+\\)['\\\"]\\{0,1\\}.*/\\1/p" \
    -e "s/.*mode='\\([^']\\+\\)'.*/\\1/p" \
    -e "s/.*mode=\\([^ ,)]\\+\\).*/\\1/p" | head -n 1 | tr -d " "
}

get_mode() {
  local state
  state=$(ros2 service call /get_robot_state orion_mtc_msgs/srv/GetRobotState "{}" 2>/dev/null || true)
  extract_mode "$state"
}

# 等待服务可用
echo ""
echo "[0] 等待 /get_robot_state 服务..."
# 避免管道导致 BrokenPipe：先缓存输出再 grep
until ros2 service list 2>/dev/null | grep -q "/get_robot_state"; do
  sleep 1
done
sleep 2

echo ""
echo "[1] 查询当前状态（预期: IDLE）"
state=$(ros2 service call /get_robot_state orion_mtc_msgs/srv/GetRobotState "{}" 2>/dev/null)
echo "$state"
mode=$(extract_mode "$state")
if [ "$mode" != "IDLE" ] && [ "$mode" != "ERROR" ]; then
  echo "  警告: 当前 mode=$mode，非 IDLE/ERROR。若为 HOLDING 请先执行一次放置或重启节点后再测。"
fi
echo "  按 Enter 继续触发「仅抓取」..."
read -r

echo ""
echo "[2] 仅抓取：发送 /pick_trigger（使用当前 /object_pose，如 target_sensor 发布的位姿）"
ros2 topic pub --once /pick_trigger std_msgs/msg/Empty "{}"
echo "  已发送。等待抓取完成（规划+执行约 20~60 秒），将轮询状态..."

max_wait=90
interval=3
elapsed=0
while [ $elapsed -lt $max_wait ]; do
  sleep $interval
  elapsed=$((elapsed + interval))
  mode=$(get_mode)
  echo "    ${elapsed}s 状态: $mode"
  if [ "$mode" = "HOLDING" ]; then
    echo "  抓取成功，状态已为 HOLDING。"
    break
  fi
  if [ "$mode" = "IDLE" ] || [ "$mode" = "ERROR" ]; then
    echo "  抓取未进入 HOLDING（当前 $mode），可能规划/执行失败，请查看 launch 终端日志。"
    exit 1
  fi
done
if [ $elapsed -ge $max_wait ]; then
  echo "  超时未进入 HOLDING，请检查日志。"
  exit 1
fi

echo ""
echo "[3] 再次查询状态（预期: HOLDING, has_held_object: true）"
ros2 service call /get_robot_state orion_mtc_msgs/srv/GetRobotState "{}" 2>/dev/null
echo "  按 Enter 继续触发「仅放置」..."
read -r

echo ""
echo "[4] 仅放置：先发 /place_pose，再发 /place_trigger"
ros2 topic pub --once /place_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.45, y: 0.0, z: 0.4}, orientation: {w: 1.0}}}"
ros2 topic pub --once /place_trigger std_msgs/msg/Empty "{}"
echo "  已发送。等待放置完成..."

elapsed=0
while [ $elapsed -lt $max_wait ]; do
  sleep $interval
  elapsed=$((elapsed + interval))
  mode=$(get_mode)
  echo "    ${elapsed}s 状态: $mode"
  if [ "$mode" = "IDLE" ]; then
    echo "  放置完成，状态已回到 IDLE。"
    break
  fi
  if [ "$mode" = "ERROR" ]; then
    echo "  放置过程进入 ERROR，请查看 launch 终端 last_error。"
    exit 1
  fi
done
if [ $elapsed -ge $max_wait ]; then
  echo "  超时未回到 IDLE，请检查日志。"
  exit 1
fi

echo ""
echo "[5] 最终状态（预期: IDLE, has_held_object: false）"
ros2 service call /get_robot_state orion_mtc_msgs/srv/GetRobotState "{}" 2>/dev/null

echo ""
echo "=============================================="
echo "  测试结束：仅抓取 -> 仅放置 逻辑按预期则通过。"
echo "=============================================="
