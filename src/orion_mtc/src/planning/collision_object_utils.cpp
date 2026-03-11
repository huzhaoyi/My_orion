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
  geometry_msgs::msg::Pose rod_local;
  rod_local.position.x = 0.0;
  rod_local.position.y = 0.0;
  rod_local.position.z = -0.10;
  rod_local.orientation.w = 1.0;
  rod_local.orientation.x = 0.0;
  rod_local.orientation.y = 0.0;
  rod_local.orientation.z = 0.0;
  geometry_msgs::msg::Pose handle_local;
  handle_local.position.x = 0.0;
  handle_local.position.y = 0.0;
  handle_local.position.z = 0.08;
  handle_local.orientation.w = 1.0;
  handle_local.orientation.x = 0.0;
  handle_local.orientation.y = 0.0;
  handle_local.orientation.z = 0.0;

  geometry_msgs::msg::Pose rod_world = composePose(object_pose, rod_local);
  geometry_msgs::msg::Pose handle_world = composePose(object_pose, handle_local);

  moveit_msgs::msg::CollisionObject object;
  object.id = object_id;
  object.header.frame_id = "base_link";
  shape_msgs::msg::SolidPrimitive rod;
  rod.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  rod.dimensions = { 0.30f, 0.015f };
  shape_msgs::msg::SolidPrimitive handle;
  handle.type = shape_msgs::msg::SolidPrimitive::BOX;
  handle.dimensions = { 0.15f, 0.03f, 0.10f };

  object.primitives.push_back(rod);
  object.primitive_poses.push_back(rod_world);
  object.primitives.push_back(handle);
  object.primitive_poses.push_back(handle_world);
  object.operation = operation;
  return object;
}

}  // namespace orion_mtc
