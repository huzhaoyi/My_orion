#include "orion_mtc/perception/vector3_cache.hpp"
#include <rclcpp/rclcpp.hpp>

namespace orion_mtc
{

namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc.perception");
}

Vector3Cache::Vector3Cache(const std::string& expected_frame_id)
  : expected_frame_id_(expected_frame_id)
{
}

void Vector3Cache::update(const geometry_msgs::msg::Vector3Stamped& msg)
{
  if (!expected_frame_id_.empty() && msg.header.frame_id != expected_frame_id_)
  {
    RCLCPP_WARN(LOGGER, "Vector3Cache: frame_id '%s' != expected '%s', ignore",
                msg.header.frame_id.c_str(), expected_frame_id_.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = msg;
    has_data_ = true;
  }
  cv_.notify_all();
}

bool Vector3Cache::hasData() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_data_;
}

std::optional<geometry_msgs::msg::Vector3Stamped> Vector3Cache::latest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_data_)
  {
    return std::nullopt;
  }
  return data_;
}

bool Vector3Cache::waitForData(std::chrono::milliseconds timeout, geometry_msgs::msg::Vector3Stamped& out) const
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::unique_lock<std::mutex> lock(mutex_);
  while (!has_data_ && std::chrono::steady_clock::now() < deadline)
  {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    cv_.wait_for(lock, std::min(remaining, std::chrono::milliseconds(50)));
  }
  if (!has_data_)
  {
    return false;
  }
  out = data_;
  return true;
}

}  // namespace orion_mtc

