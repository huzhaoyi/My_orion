/* 纯几何/碰撞体工具：composePose、makeTargetCollisionObject，非业务逻辑 */

#ifndef ORION_MTC_PLANNING_COLLISION_OBJECT_UTILS_HPP
#define ORION_MTC_PLANNING_COLLISION_OBJECT_UTILS_HPP

#include "orion_mtc/planning/cable_segments.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <cstdint>

namespace orion_mtc
{

/* 局部 pose 在整体 pose 下变换：result = base * local（世界系下的 local） */
geometry_msgs::msg::Pose composePose(const geometry_msgs::msg::Pose& base,
                                      const geometry_msgs::msg::Pose& local);

/* 统一生成 target 组合碰撞体：杆 + 把手框，primitive_poses 为世界系（base_link） */
moveit_msgs::msg::CollisionObject makeTargetCollisionObject(const std::string& object_id,
                                                             const geometry_msgs::msg::Pose& object_pose,
                                                             uint8_t operation);

/* 单段缆绳短圆柱碰撞体：局部 Z 对齐 segment.axis，中心在 segment.center */
moveit_msgs::msg::CollisionObject makeSegmentCollisionObject(const CableSegment& segment,
                                                              const std::string& frame_id,
                                                              uint8_t operation);

}  // namespace orion_mtc

#endif  // ORION_MTC_PLANNING_COLLISION_OBJECT_UTILS_HPP
