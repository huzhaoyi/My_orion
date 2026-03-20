/* 放置候选：与抓取对称，支持多候选依次规划 */

#ifndef ORION_MTC_CORE_PLACE_TYPES_HPP
#define ORION_MTC_CORE_PLACE_TYPES_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

namespace orion_mtc
{

/** 单条放置候选：物体目标位姿 + TCP 位姿 + 打分与来源 */
struct PlaceCandidate
{
  geometry_msgs::msg::PoseStamped object_pose;
  geometry_msgs::msg::Pose tcp_pose;
  double score = 0.0;
  std::string source;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_CORE_PLACE_TYPES_HPP
