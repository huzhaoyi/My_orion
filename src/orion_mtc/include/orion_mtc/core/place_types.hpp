/* 放置目标与候选：与抓取对称，支持“放置位选择 + 多候选尝试” */

#ifndef ORION_MTC_CORE_PLACE_TYPES_HPP
#define ORION_MTC_CORE_PLACE_TYPES_HPP

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <string>

namespace orion_mtc
{

/** 放置模式：精确物体位姿 或 仅 TCP 释放位 */
enum class PlaceMode
{
  PRECISE_OBJECT_POSE,
  RELEASE_TCP_POSE
};

/** 单条放置候选：物体目标位姿 + 可选 TCP 位姿 + 打分与来源 */
struct PlaceCandidate
{
  geometry_msgs::msg::PoseStamped object_pose;
  geometry_msgs::msg::Pose tcp_pose;
  double score = 0.0;
  std::string source;
};

/** 放置目标：可带区域与轴向，供 PlaceGenerator 采样 */
struct PlaceTarget
{
  geometry_msgs::msg::Point position;
  geometry_msgs::msg::Vector3 axis_direction;
  bool has_axis_direction = false;
  double region_radius_xy = 0.0;
  double z_tolerance = 0.0;
};

/** 释放区域（release place）：中心 + 平面范围 + 高度 */
struct ReleaseRegion
{
  geometry_msgs::msg::Point center;
  double size_x = 0.0;
  double size_y = 0.0;
  double z = 0.0;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_CORE_PLACE_TYPES_HPP
