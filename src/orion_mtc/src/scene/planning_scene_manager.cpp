#include "orion_mtc/scene/planning_scene_manager.hpp"
#include "orion_mtc/planning/collision_object_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <chrono>

namespace orion_mtc
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc.scene");

PlanningSceneManager::PlanningSceneManager(rclcpp::Node* node) : node_(node)
{
}

bool PlanningSceneManager::ensureClient()
{
  if (!node_)
  {
    return false;
  }
  if (!apply_planning_scene_client_)
  {
    apply_planning_scene_client_ =
        node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
  }
  if (!apply_planning_scene_client_->service_is_ready())
  {
    if (!apply_planning_scene_client_->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_WARN(LOGGER, "apply_planning_scene not available");
      return false;
    }
  }
  return true;
}

bool PlanningSceneManager::applySceneDiff(const moveit_msgs::msg::PlanningScene& scene_diff)
{
  if (!ensureClient())
  {
    return false;
  }
  auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  req->scene = scene_diff;
  auto fut = apply_planning_scene_client_->async_send_request(req);
  if (fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
  {
    RCLCPP_WARN(LOGGER, "apply_planning_scene timed out");
    return false;
  }
  auto res = fut.get();
  if (!res || !res->success)
  {
    RCLCPP_WARN(LOGGER, "apply_planning_scene returned false");
    return false;
  }
  return true;
}

bool PlanningSceneManager::applyObjectPoseToPlanningScene(double px, double py, double pz,
                                                           double qx, double qy, double qz, double qw)
{
  if (!ensureClient())
  {
    RCLCPP_WARN(LOGGER, "applyObjectPoseToPlanningScene: apply_planning_scene not available");
    return false;
  }
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  moveit_msgs::msg::CollisionObject remove_obj;
  remove_obj.id = "object";
  remove_obj.header.frame_id = "base_link";
  remove_obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  scene.world.collision_objects.push_back(remove_obj);
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = px;
  target_pose.position.y = py;
  target_pose.position.z = pz;
  target_pose.orientation.x = qx;
  target_pose.orientation.y = qy;
  target_pose.orientation.z = qz;
  target_pose.orientation.w = qw;
  moveit_msgs::msg::CollisionObject add_obj =
      makeTargetCollisionObject("object", target_pose, moveit_msgs::msg::CollisionObject::ADD);
  add_obj.header.stamp = node_->now();
  scene.world.collision_objects.push_back(add_obj);
  auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  req->scene = scene;
  auto fut = apply_planning_scene_client_->async_send_request(req);
  if (fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
  {
    RCLCPP_WARN(LOGGER, "applyObjectPoseToPlanningScene: timed out");
    return false;
  }
  auto res = fut.get();
  if (!res || !res->success)
  {
    RCLCPP_WARN(LOGGER, "applyObjectPoseToPlanningScene: apply returned false");
    return false;
  }
  RCLCPP_INFO(LOGGER, "applyObjectPoseToPlanningScene: object updated to (%.3f, %.3f, %.3f)", px, py, pz);
  return true;
}

bool PlanningSceneManager::applyAttachedHeldUnknownToScene()
{
  if (!ensureClient())
  {
    RCLCPP_WARN(LOGGER, "applyAttachedHeldUnknownToScene: apply_planning_scene not available");
    return false;
  }
  moveit_msgs::msg::AttachedCollisionObject att;
  att.link_name = "Link6";
  att.object.id = "held_unknown";
  att.object.header.frame_id = "Link6";
  att.object.header.stamp = node_->now();
  att.object.pose.position.x = 0.0;
  att.object.pose.position.y = 0.0;
  att.object.pose.position.z = 0.0;
  att.object.pose.orientation.w = 1.0;
  att.object.pose.orientation.x = 0.0;
  att.object.pose.orientation.y = 0.0;
  att.object.pose.orientation.z = 0.0;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = { 0.08f, 0.06f, 0.25f };
  geometry_msgs::msg::Pose prim_pose;
  prim_pose.position.x = 0.0;
  prim_pose.position.y = 0.0;
  prim_pose.position.z = -0.12f;
  prim_pose.orientation.w = 1.0;
  prim_pose.orientation.x = 0.0;
  prim_pose.orientation.y = 0.0;
  prim_pose.orientation.z = 0.0;
  att.object.primitives.push_back(box);
  att.object.primitive_poses.push_back(prim_pose);
  att.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  att.touch_links = { "Link6", "Link7", "Link8" };

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;  // 仅附加 attached 变更，不覆盖 joint_state
  scene.robot_state.attached_collision_objects.push_back(att);
  auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  req->scene = scene;
  auto fut = apply_planning_scene_client_->async_send_request(req);
  if (fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
  {
    RCLCPP_WARN(LOGGER, "applyAttachedHeldUnknownToScene: timed out");
    return false;
  }
  auto res = fut.get();
  if (!res || !res->success)
  {
    RCLCPP_WARN(LOGGER, "applyAttachedHeldUnknownToScene: apply returned false");
    return false;
  }
  RCLCPP_INFO(LOGGER, "applyAttachedHeldUnknownToScene: attached held_unknown to Link6");
  return true;
}

bool PlanningSceneManager::clearAttachedObjectFromPlanningScene(const std::string& object_id)
{
  if (object_id.empty())
  {
    return true;
  }
  if (!ensureClient())
  {
    RCLCPP_WARN(LOGGER, "clearAttachedObjectFromPlanningScene: apply_planning_scene not available");
    return false;
  }
  moveit_msgs::msg::AttachedCollisionObject att;
  /* held_unknown 包络在 Link6；held_tracked / object 与规划 TCP 一致，挂在 gripper_tcp */
  att.link_name = (object_id == "held_unknown") ? "Link6" : "gripper_tcp";
  att.object.id = object_id;
  att.object.header.frame_id = "base_link";
  att.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;  // 仅附加 detach 变更，避免空 JointState 覆盖当前状态
  scene.robot_state.attached_collision_objects.push_back(att);
  auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  req->scene = scene;
  auto fut = apply_planning_scene_client_->async_send_request(req);
  if (fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
  {
    RCLCPP_WARN(LOGGER, "clearAttachedObjectFromPlanningScene: timed out");
    return false;
  }
  auto res = fut.get();
  if (!res || !res->success)
  {
    RCLCPP_WARN(LOGGER, "clearAttachedObjectFromPlanningScene: apply returned false for id=%s",
                object_id.c_str());
    return false;
  }
  RCLCPP_INFO(LOGGER, "clearAttachedObjectFromPlanningScene: cleared id=%s", object_id.c_str());
  return true;
}

bool PlanningSceneManager::removeWorldObject(const std::string& object_id)
{
  if (object_id.empty())
  {
    return true;
  }
  if (!ensureClient())
  {
    RCLCPP_WARN(LOGGER, "removeWorldObject: apply_planning_scene not available");
    return false;
  }
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  moveit_msgs::msg::CollisionObject obj;
  obj.id = object_id;
  obj.header.frame_id = "base_link";
  obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  scene.world.collision_objects.push_back(obj);
  if (!applySceneDiff(scene))
  {
    RCLCPP_WARN(LOGGER, "removeWorldObject: apply failed for id=%s", object_id.c_str());
    return false;
  }
  RCLCPP_INFO(LOGGER, "removeWorldObject: removed world id=%s", object_id.c_str());
  return true;
}

bool PlanningSceneManager::applyAttachedTrackedObjectToScene(const Eigen::Isometry3d& tcp_to_object,
                                                             double grasp_offset_along_axis)
{
  if (!ensureClient())
  {
    RCLCPP_WARN(LOGGER, "applyAttachedTrackedObjectToScene: apply_planning_scene not available");
    return false;
  }
  /* tcp_to_object = T_tcp_obj：位姿在 gripper_tcp 系；附加重载也用 gripper_tcp，避免漏乘 T_link6_tcp */
  Eigen::Vector3d center_in_tcp = tcp_to_object.translation();
  if (std::abs(grasp_offset_along_axis) > 1e-6)
  {
    Eigen::Vector3d cable_axis_in_tcp = tcp_to_object.rotation().inverse() * Eigen::Vector3d::UnitZ();
    center_in_tcp += (-grasp_offset_along_axis) * cable_axis_in_tcp;
  }
  geometry_msgs::msg::Pose tcp_object_pose;
  tcp_object_pose.position.x = center_in_tcp.x();
  tcp_object_pose.position.y = center_in_tcp.y();
  tcp_object_pose.position.z = center_in_tcp.z();
  Eigen::Quaterniond q_rot(tcp_to_object.rotation());
  tcp_object_pose.orientation.x = q_rot.x();
  tcp_object_pose.orientation.y = q_rot.y();
  tcp_object_pose.orientation.z = q_rot.z();
  tcp_object_pose.orientation.w = q_rot.w();
  geometry_msgs::msg::Pose rod_local;
  rod_local.position.x = 0.0;
  rod_local.position.y = 0.0;
  rod_local.position.z = 0.0;
  rod_local.orientation.w = 1.0;
  rod_local.orientation.x = 0.0;
  rod_local.orientation.y = 0.0;
  rod_local.orientation.z = 0.0;
  geometry_msgs::msg::Pose rod_in_tcp = composePose(tcp_object_pose, rod_local);

  moveit_msgs::msg::AttachedCollisionObject att;
  att.link_name = "gripper_tcp";
  att.object.id = "held_tracked";
  att.object.header.frame_id = "gripper_tcp";
  att.object.header.stamp = node_->now();
  att.object.pose.position.x = 0.0;
  att.object.pose.position.y = 0.0;
  att.object.pose.position.z = 0.0;
  att.object.pose.orientation.w = 1.0;
  att.object.pose.orientation.x = 0.0;
  att.object.pose.orientation.y = 0.0;
  att.object.pose.orientation.z = 0.0;
  shape_msgs::msg::SolidPrimitive rod;
  rod.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  // 缆绳建模：3m 长、直径 5cm 的圆柱体（MoveIt SolidPrimitive::CYLINDER: [height, radius]）
  rod.dimensions = { 3.0f, 0.025f };
  att.object.primitives.push_back(rod);
  att.object.primitive_poses.push_back(rod_in_tcp);
  att.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  att.touch_links = { "gripper_tcp", "Link6", "Link7", "Link8" };

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;  // 仅附加 attached 变更，不覆盖 joint_state
  scene.robot_state.attached_collision_objects.push_back(att);
  auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  req->scene = scene;
  auto fut = apply_planning_scene_client_->async_send_request(req);
  if (fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
  {
    RCLCPP_WARN(LOGGER, "applyAttachedTrackedObjectToScene: timed out");
    return false;
  }
  auto res = fut.get();
  if (!res || !res->success)
  {
    RCLCPP_WARN(LOGGER, "applyAttachedTrackedObjectToScene: apply returned false");
    return false;
  }
  RCLCPP_INFO(LOGGER, "applyAttachedTrackedObjectToScene: attached held_tracked to gripper_tcp");
  return true;
}

}  // namespace orion_mtc
