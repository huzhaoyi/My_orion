/* 感知输入缓存层：收消息、校验 frame_id、保存最后一帧、线程安全读取、wait/latest/has_data */

#ifndef ORION_MTC_PERCEPTION_POSE_CACHE_HPP
#define ORION_MTC_PERCEPTION_POSE_CACHE_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>

namespace orion_mtc
{

class PoseCache
{
public:
  /** 期望的 frame_id，默认 "base_link"；update 时若不一致则忽略并打 WARN */
  explicit PoseCache(const std::string& expected_frame_id = "base_link");
  void update(const geometry_msgs::msg::PoseStamped& msg);
  bool hasPose() const;
  std::optional<geometry_msgs::msg::PoseStamped> latest() const;
  /** 阻塞等待直到有数据或超时；out 仅在返回 true 时有效 */
  bool waitForPose(std::chrono::milliseconds timeout,
                   geometry_msgs::msg::PoseStamped& out) const;
  /** 阻塞直到发生一次 update 或超时，便于调用方在取 latest() 前等到“更新后”的位姿 */
  void waitForNextUpdate(std::chrono::milliseconds timeout) const;

private:
  std::string expected_frame_id_;
  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  geometry_msgs::msg::PoseStamped pose_;
  bool has_pose_ = false;
  uint64_t update_count_ = 0;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_PERCEPTION_POSE_CACHE_HPP
