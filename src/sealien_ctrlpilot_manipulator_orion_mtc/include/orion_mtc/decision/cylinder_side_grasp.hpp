/* 圆柱碰撞体姿态构造：MoveIt CYLINDER 的局部 Z 对齐到 axis_direction */

#ifndef ORION_MTC_DECISION_CYLINDER_SIDE_GRASP_HPP
#define ORION_MTC_DECISION_CYLINDER_SIDE_GRASP_HPP

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace orion_mtc
{

/* 圆柱碰撞体姿态：MoveIt CYLINDER 的局部 Z 对齐到 axis_direction */
geometry_msgs::msg::Quaternion buildCylinderCollisionOrientationFromAxis(
    const geometry_msgs::msg::Vector3& axis_direction);

}  // namespace orion_mtc

#endif  // ORION_MTC_DECISION_CYLINDER_SIDE_GRASP_HPP

