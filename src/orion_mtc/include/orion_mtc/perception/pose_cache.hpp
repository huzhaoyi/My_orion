/* 感知输入缓存层：收消息、校验 frame_id、保存最后一帧、线程安全读取、wait/latest/has_data */

#ifndef ORION_MTC_PERCEPTION_POSE_CACHE_HPP
#define ORION_MTC_PERCEPTION_POSE_CACHE_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>
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

private:
  std::string expected_frame_id_;
  mutable std::mutex mutex_;
  geometry_msgs::msg::PoseStamped pose_;
  bool has_pose_ = false;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_PERCEPTION_POSE_CACHE_HPP
