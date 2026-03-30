/* pose_cache：object_pose 订阅侧校验 frame、最近帧与条件等待 */

#include "orion_mtc/perception/pose_cache.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace orion_mtc
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc.perception");

/*
 * expected_frame_id 非空时丢弃 frame 不一致的消息，减少错误系混用。
 */
PoseCache::PoseCache(const std::string& expected_frame_id) : expected_frame_id_(expected_frame_id)
{
}

/*
 * 更新最新 pose、递增计数并 notify 条件变量；帧校验失败则 WARN 返回。
 */
void PoseCache::update(const geometry_msgs::msg::PoseStamped& msg)
{
  if (!expected_frame_id_.empty() && msg.header.frame_id != expected_frame_id_)
  {
    RCLCPP_WARN(LOGGER, "pose frame_id is '%s', expected %s; ignore",
                msg.header.frame_id.c_str(), expected_frame_id_.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_ = msg;
    has_pose_ = true;
    ++update_count_;
    cv_.notify_all();
  }
  RCLCPP_DEBUG(LOGGER, "pose received: (%.3f, %.3f, %.3f)",
               msg.pose.position.x, msg.pose.position.y,                msg.pose.position.z);
}

/*
 * 是否已至少收到一帧合法 pose。
 */
bool PoseCache::hasPose() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_pose_;
}

/*
 * 快照拷贝当前 pose；无数据返回 nullopt。
 */
std::optional<geometry_msgs::msg::PoseStamped> PoseCache::latest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_pose_)
    return std::nullopt;
  return pose_;
}

/*
 * 轮询式超时等待：适合无 executor 线程仅 sleep 检查 has_pose_ 的场景。
 */
bool PoseCache::waitForPose(std::chrono::milliseconds timeout,
                           geometry_msgs::msg::PoseStamped& out) const
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  const auto tick = std::chrono::milliseconds(50);
  while (std::chrono::steady_clock::now() < deadline)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_pose_)
      {
        out = pose_;
        return true;
      }
    }
    std::this_thread::sleep_for(tick);
  }
  return false;
}

/*
 * 条件变量等待 update_count_ 相对调用时刻增长，用于「等到下一帧 perception」再规划。
 */
bool PoseCache::waitForNextUpdate(std::chrono::milliseconds timeout) const
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::unique_lock<std::mutex> lock(mutex_);
  const uint64_t count_at_start = update_count_;
  while (update_count_ == count_at_start && std::chrono::steady_clock::now() < deadline)
  {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline -
                                                                                 std::chrono::steady_clock::now());
    cv_.wait_for(lock, std::min(remaining, std::chrono::milliseconds(50)));
  }
  return update_count_ != count_at_start;
}

}  // namespace orion_mtc
