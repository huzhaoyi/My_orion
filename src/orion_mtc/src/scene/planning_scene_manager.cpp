/* planning_scene_manager：apply_planning_scene 封装，world 与 attach 增删 */

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
static constexpr int APPLY_PLANNING_SCENE_WAIT_SEC = 30;

/*
 * 仅保存 node 指针；apply_planning_scene 客户端按需惰性创建。
 */
PlanningSceneManager::PlanningSceneManager(rclcpp::Node::SharedPtr node,
                                            rclcpp::CallbackGroup::SharedPtr reentrant_client_group)
  : node_shared_(std::move(node)), reentrant_client_group_(std::move(reentrant_client_group))
{
}

/*
 * 创建 /apply_planning_scene 客户端并在未就绪时短等待；node 为空或服务不可用返回 false。
 */
bool PlanningSceneManager::ensureClient()
{
  if (!node_shared_)
  {
    return false;
  }
  if (!apply_planning_scene_client_)
  {
    // Reentrant：go_to_ready 等在服务回调里同步 wait future 时，默认 MutuallyExclusive 会卡死异步响应
    apply_planning_scene_client_ = node_shared_->create_client<moveit_msgs::srv::ApplyPlanningScene>(
        "/apply_planning_scene", rmw_qos_profile_services_default, reentrant_client_group_);
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

/*
 * 将 is_diff 场景整块交给 apply_planning_scene；同步等待响应，超时或服务失败记 WARN 并返回 false。
 */
bool PlanningSceneManager::applySceneDiff(const moveit_msgs::msg::PlanningScene& scene_diff)
{
  if (!ensureClient())
  {
    return false;
  }
  auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  req->scene = scene_diff;
  auto fut = apply_planning_scene_client_->async_send_request(req);
  if (fut.wait_for(std::chrono::seconds(APPLY_PLANNING_SCENE_WAIT_SEC)) != std::future_status::ready)
  {
    RCLCPP_WARN(LOGGER, "apply_planning_scene timed out (%ds)", APPLY_PLANNING_SCENE_WAIT_SEC);
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

/*
 * 先 REMOVE 再 ADD id=object 的圆柱目标体，位姿由 px..qw 给定；用于 perception 更新 world 物体。
 */
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
  add_obj.header.stamp = node_shared_->now();
  scene.world.collision_objects.push_back(add_obj);
  auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  req->scene = scene;
  auto fut = apply_planning_scene_client_->async_send_request(req);
  if (fut.wait_for(std::chrono::seconds(APPLY_PLANNING_SCENE_WAIT_SEC)) != std::future_status::ready)
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

/*
 * 在 Link6 附加 held_unknown 盒体，表示未知形状持物占位；touch_links 限制与手爪链路一致。
 */
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
  att.object.header.stamp = node_shared_->now();
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
  if (fut.wait_for(std::chrono::seconds(APPLY_PLANNING_SCENE_WAIT_SEC)) != std::future_status::ready)
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

/*
 * 对指定 id 发送 REMOVE 式 AttachedCollisionObject；held_unknown 挂 Link6，其余默认 gripper_tcp。
 * robot_state.is_diff 避免空 JointState 覆盖当前机器人状态。
 */
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
  if (fut.wait_for(std::chrono::seconds(APPLY_PLANNING_SCENE_WAIT_SEC)) != std::future_status::ready)
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

/*
 * world 碰撞体按 id REMOVE。MoveIt 对「移除场景中不存在的 id」常返回 success=false，
 * 对清理路径应幂等成功，故仅在超时/无服务时返回 false。
 */
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
  auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  req->scene = scene;
  auto fut = apply_planning_scene_client_->async_send_request(req);
  if (fut.wait_for(std::chrono::seconds(APPLY_PLANNING_SCENE_WAIT_SEC)) != std::future_status::ready)
  {
    RCLCPP_WARN(LOGGER, "removeWorldObject: timed out for id=%s", object_id.c_str());
    return false;
  }
  auto res = fut.get();
  if (!res || !res->success)
  {
    RCLCPP_DEBUG(LOGGER, "removeWorldObject: apply returned false for id=%s (absent or sync; idempotent ok)",
                 object_id.c_str());
    return true;
  }
  RCLCPP_INFO(LOGGER, "removeWorldObject: removed world id=%s", object_id.c_str());
  return true;
}

/*
 * tcp_to_object 为 TCP 到物体中心的刚体变换；可选沿缆轴 grasp_offset_along_axis 平移中心。
 * 圆柱 primitive pose 在 TCP 系组合后附加为 held_tracked，供 TRACK 与后续规划一致。
 */
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
  att.object.header.stamp = node_shared_->now();
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
  if (fut.wait_for(std::chrono::seconds(APPLY_PLANNING_SCENE_WAIT_SEC)) != std::future_status::ready)
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
