#include "orion_mtc/perception/target_cache.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <algorithm>

namespace orion_mtc
{

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc.perception");
}

TargetCache::TargetCache(const std::string& expected_frame_id) : expected_frame_id_(expected_frame_id)
{
}

void TargetCache::update(const orion_mtc_msgs::msg::TargetSet& msg)
{
  const int n = static_cast<int>(msg.num_targets);
  if (n <= 0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    targets_.clear();
    has_targets_ = false;
    return;
  }
  if (!expected_frame_id_.empty() && msg.header.frame_id != expected_frame_id_)
  {
    RCLCPP_WARN(LOGGER, "TargetCache: frame_id '%s' != expected '%s', ignore",
                msg.header.frame_id.c_str(), expected_frame_id_.c_str());
    return;
  }
  const size_t pos_len = msg.positions.size();
  const size_t dir_len = msg.directions.size();
  if (pos_len < static_cast<size_t>(3 * n) || dir_len < static_cast<size_t>(3 * n))
  {
    RCLCPP_WARN(LOGGER, "TargetCache: num_targets=%d but positions.size=%zu, directions.size=%zu",
                n, pos_len, dir_len);
    return;
  }

  const int64_t stamp_ns = msg.header.stamp.sec * 1000000000LL + msg.header.stamp.nanosec;
  std::vector<TargetObject> out;
  out.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i)
  {
    TargetObject obj;
    obj.index = i;
    const size_t o = static_cast<size_t>(i * 3);
    obj.position.x = msg.positions[o];
    obj.position.y = msg.positions[o + 1];
    obj.position.z = msg.positions[o + 2];
    double dx = msg.directions[o];
    double dy = msg.directions[o + 1];
    double dz = msg.directions[o + 2];
    double norm = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (norm < 1e-9)
    {
      obj.axis_direction.x = 0.0;
      obj.axis_direction.y = 0.0;
      obj.axis_direction.z = 1.0;
    }
    else
    {
      obj.axis_direction.x = dx / norm;
      obj.axis_direction.y = dy / norm;
      obj.axis_direction.z = dz / norm;
    }
    obj.object_type = "target";
    obj.stable_id = -1;
    obj.confidence = 1.0;
    obj.stamp_ns = stamp_ns;
    out.push_back(obj);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  targets_ = std::move(out);
  has_targets_ = !targets_.empty();
}

std::vector<TargetObject> TargetCache::latest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return targets_;
}

bool TargetCache::hasTargets() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_targets_;
}

}  // namespace orion_mtc
