#include "orion_mtc/planning/pick_task_builder.hpp"
#include "orion_mtc/planning/collision_object_utils.hpp"
#include "orion_mtc/core/constants.hpp"
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mtc = moveit::task_constructor;

namespace orion_mtc
{

PickTaskBuilder::PickTaskBuilder(const rclcpp::Node::SharedPtr& node, const MTCConfig& config)
  : node_(node), config_(config)
{
}

mtc::Task PickTaskBuilder::build(double obj_x, double obj_y, double obj_z,
                                 const geometry_msgs::msg::Quaternion& object_orientation,
                                 const std::string& /*object_id*/)
{
  mtc::Task task;
  task.stages()->setName("orion pick");
  task.loadRobotModel(node_);
  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "Link6";
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.stages()->properties().set("group", arm_group_name);
  task.stages()->properties().set("eef", hand_group_name);
  task.stages()->properties().set("ik_frame", hand_frame);

  task.add(std::make_unique<mtc::stages::CurrentState>("current"));
  {
    geometry_msgs::msg::Pose obj_pose;
    obj_pose.position.x = obj_x;
    obj_pose.position.y = obj_y;
    obj_pose.position.z = obj_z;
    obj_pose.orientation = object_orientation;
    moveit_msgs::msg::CollisionObject object =
        makeTargetCollisionObject("object", obj_pose, moveit_msgs::msg::CollisionObject::ADD);
    object.header.stamp = node_->now();
    auto stage_add = std::make_unique<mtc::stages::ModifyPlanningScene>("add object");
    stage_add->addObject(object);
    task.add(std::move(stage_add));
  }

  auto ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  ptp_planner->setPlannerId("PTP");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  auto stage_ready = std::make_unique<mtc::stages::MoveTo>("move to ready", ptp_planner);
  stage_ready->setGroup(arm_group_name);
  stage_ready->setGoal("ready");
  task.add(std::move(stage_ready));

  auto stage_open = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open->setGroup(hand_group_name);
  stage_open->setGoal("open");
  task.add(std::move(stage_open));

  {
    auto stage_acm = std::make_unique<mtc::stages::ModifyPlanningScene>("allow self-collision (pregrasp)");
    stage_acm->allowCollisions("Link1", std::vector<std::string>{ "Link8" }, true);
    stage_acm->allowCollisions("Link2", std::vector<std::string>{ "Link8" }, true);
    stage_acm->allowCollisions("Link7", std::vector<std::string>{ "base_link" }, true);
    stage_acm->allowCollisions("Link8", std::vector<std::string>{ "base_link" }, true);
    task.add(std::move(stage_acm));
  }

  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

  const double approach_max = config_.approach_object_max_dist;
  geometry_msgs::msg::PoseStamped pregrasp;
  pregrasp.header.frame_id = "base_link";
  pregrasp.pose.position.x = obj_x;
  pregrasp.pose.position.y = obj_y;
  pregrasp.pose.position.z = obj_z + approach_max;
  /* 与抓取候选一致：预抓取朝向使用物体朝向，避免固定 (1,0,0,0) 在第二次或不同目标下无 IK 解 */
  pregrasp.pose.orientation = object_orientation;
  auto stage_pregrasp = std::make_unique<mtc::stages::MoveTo>("move to pregrasp", ptp_planner);
  stage_pregrasp->setGroup(arm_group_name);
  stage_pregrasp->setGoal(pregrasp);
  stage_pregrasp->setIKFrame(hand_frame);
  grasp->insert(std::move(stage_pregrasp));

  /* 只在接近/闭合/attach 阶段临时允许 hand-object，不提前放开避免 move to pregrasp 穿物 */
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    stage->allowCollisions("object", OBJECT_GRASP_ALLOWED_LINKS, true);
    grasp->insert(std::move(stage));
  }

  auto stage_approach = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
  stage_approach->properties().set("marker_ns", "approach_object");
  stage_approach->properties().set("link", hand_frame);
  stage_approach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_approach->properties().set("min_fraction", 0.55);
  stage_approach->setMinMaxDistance(static_cast<float>(config_.approach_object_min_dist),
                                   static_cast<float>(config_.approach_object_max_dist));
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = hand_frame;
  vec.vector.x = 0.0;
  vec.vector.y = 0.0;
  vec.vector.z = 1.0;
  stage_approach->setDirection(vec);
  grasp->insert(std::move(stage_approach));

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision before close");
    stage->allowCollisions("object", OBJECT_GRASP_ALLOWED_LINKS, true);
    grasp->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("close");
    grasp->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject("object", hand_frame);
    grasp->insert(std::move(stage));
  }

  if (!config_.support_surface_link.empty())
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,support)");
    stage->allowCollisions("object", std::vector<std::string>{ config_.support_surface_link }, true);
    grasp->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(static_cast<float>(config_.lift_object_min_dist),
                             static_cast<float>(config_.lift_object_max_dist));
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "lift_object");
    geometry_msgs::msg::Vector3Stamped v;
    v.header.frame_id = "base_link";
    v.vector.x = 0.0;
    v.vector.y = 0.0;
    v.vector.z = 1.0;
    stage->setDirection(v);
    grasp->insert(std::move(stage));
  }

  if (!config_.support_surface_link.empty())
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object,surface)");
    stage->allowCollisions("object", std::vector<std::string>{ config_.support_surface_link }, false);
    grasp->insert(std::move(stage));
  }

  task.add(std::move(grasp));
  return task;
}

}  // namespace orion_mtc
