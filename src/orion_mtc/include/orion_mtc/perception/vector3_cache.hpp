/* 轻量 Vector3Stamped 缓存：用于 object_axis 等方向向量话题 */

#ifndef ORION_MTC_PERCEPTION_VECTOR3_CACHE_HPP
#define ORION_MTC_PERCEPTION_VECTOR3_CACHE_HPP

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>

namespace orion_mtc
{

class Vector3Cache
{
public:
  explicit Vector3Cache(const std::string& expected_frame_id = "base_link");
  void update(const geometry_msgs::msg::Vector3Stamped& msg);
  bool hasData() const;
  std::optional<geometry_msgs::msg::Vector3Stamped> latest() const;
  bool waitForData(std::chrono::milliseconds timeout, geometry_msgs::msg::Vector3Stamped& out) const;

private:
  std::string expected_frame_id_;
  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  geometry_msgs::msg::Vector3Stamped data_;
  bool has_data_ = false;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_PERCEPTION_VECTOR3_CACHE_HPP

