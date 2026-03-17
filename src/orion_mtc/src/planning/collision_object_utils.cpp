#include "orion_mtc/planning/collision_object_utils.hpp"
#include <shape_msgs/msg/solid_primitive.hpp>
#include <Eigen/Geometry>

namespace orion_mtc
{

geometry_msgs::msg::Pose composePose(const geometry_msgs::msg::Pose& base,
                                      const geometry_msgs::msg::Pose& local)
{
  Eigen::Isometry3d T_base = Eigen::Isometry3d::Identity();
  T_base.translate(Eigen::Vector3d(base.position.x, base.position.y, base.position.z));
  T_base.rotate(Eigen::Quaterniond(base.orientation.w, base.orientation.x, base.orientation.y,
                                   base.orientation.z));
  Eigen::Isometry3d T_local = Eigen::Isometry3d::Identity();
  T_local.translate(Eigen::Vector3d(local.position.x, local.position.y, local.position.z));
  T_local.rotate(Eigen::Quaterniond(local.orientation.w, local.orientation.x, local.orientation.y,
                                    local.orientation.z));
  Eigen::Isometry3d T = T_base * T_local;
  geometry_msgs::msg::Pose out;
  out.position.x = T.translation().x();
  out.position.y = T.translation().y();
  out.position.z = T.translation().z();
  Eigen::Quaterniond q(T.rotation());
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
  out.orientation.w = q.w();
  return out;
}

moveit_msgs::msg::CollisionObject makeTargetCollisionObject(const std::string& object_id,
                                                            const geometry_msgs::msg::Pose& object_pose,
                                                            uint8_t operation)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = object_id;
  object.header.frame_id = "base_link";

  // 缆绳建模：3m 长、直径 5cm 的圆柱体（MoveIt SolidPrimitive::CYLINDER: [height, radius]）
  shape_msgs::msg::SolidPrimitive cable;
  cable.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cable.dimensions = { 3.0f, 0.025f };
  object.primitives.push_back(cable);
  object.primitive_poses.push_back(object_pose);
  object.operation = operation;
  return object;
}

}  // namespace orion_mtc
