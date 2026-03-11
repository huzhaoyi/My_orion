#include "orion_mtc/planning/place_release_task_builder.hpp"
#include "orion_mtc/core/constants.hpp"
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mtc = moveit::task_constructor;

namespace orion_mtc
{

PlaceReleaseTaskBuilder::PlaceReleaseTaskBuilder(const rclcpp::Node::SharedPtr& node,
                                                  const MTCConfig& config)
  : node_(node), config_(config)
{
}

mtc::Task PlaceReleaseTaskBuilder::build(const geometry_msgs::msg::PoseStamped& target_tcp_pose,
                                         const std::string& attached_object_id)
{
  const double lower_max = config_.lower_to_place_max_dist;
  double tx = target_tcp_pose.pose.position.x;
  double ty = target_tcp_pose.pose.position.y;
  double tz = target_tcp_pose.pose.position.z;
  const auto& q = target_tcp_pose.pose.orientation;
  geometry_msgs::msg::Pose pre_place_pose;
  pre_place_pose.position.x = tx;
  pre_place_pose.position.y = ty;
  pre_place_pose.position.z = tz + lower_max;
  pre_place_pose.orientation = q;

  mtc::Task task;
  task.stages()->setName("orion place release");
  task.loadRobotModel(node_);
  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "Link6";
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  task.add(std::make_unique<mtc::stages::CurrentState>("current"));

  if (!attached_object_id.empty())
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    stage->allowCollisions(attached_object_id, OBJECT_GRASP_ALLOWED_LINKS, true);
    task.add(std::move(stage));
  }

  auto ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  ptp_planner->setPlannerId("PTP");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  geometry_msgs::msg::PoseStamped pre_stamped;
  pre_stamped.header.frame_id =
      target_tcp_pose.header.frame_id.empty() ? "base_link" : target_tcp_pose.header.frame_id;
  pre_stamped.pose = pre_place_pose;
  auto stage_pre = std::make_unique<mtc::stages::MoveTo>("move to pre-place (release)", ptp_planner);
  stage_pre->setGroup(arm_group_name);
  stage_pre->setGoal(pre_stamped);
  stage_pre->setIKFrame(hand_frame);
  task.add(std::move(stage_pre));

  auto place = std::make_unique<mtc::SerialContainer>("place release");
  place->properties().set("eef", hand_group_name);
  place->properties().set("group", arm_group_name);
  place->properties().set("ik_frame", hand_frame);

  auto stage_lower = std::make_unique<mtc::stages::MoveRelative>("lower to place", cartesian_planner);
  stage_lower->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_lower->setMinMaxDistance(static_cast<float>(config_.lower_to_place_min_dist),
                                static_cast<float>(config_.lower_to_place_max_dist));
  stage_lower->setIKFrame(hand_frame);
  stage_lower->properties().set("marker_ns", "lower_release");
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = hand_frame;
  vec.vector.x = 0.0;
  vec.vector.y = 0.0;
  vec.vector.z = -1.0;
  stage_lower->setDirection(vec);
  place->insert(std::move(stage_lower));

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("open");
    place->insert(std::move(stage));
  }

  if (attached_object_id == "object" || attached_object_id == "held_tracked")
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage->detachObject(attached_object_id, hand_frame);
    place->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(static_cast<float>(config_.retreat_min_dist),
                            static_cast<float>(config_.retreat_max_dist));
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "retreat");
    geometry_msgs::msg::Vector3Stamped v;
    v.header.frame_id = hand_frame;
    v.vector.x = 0.0;
    v.vector.y = 0.0;
    v.vector.z = -1.0;
    stage->setDirection(v);
    place->insert(std::move(stage));
  }

  if (!attached_object_id.empty())
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
    stage->allowCollisions(attached_object_id, OBJECT_GRASP_ALLOWED_LINKS, false);
    place->insert(std::move(stage));
  }

  task.add(std::move(place));

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", ptp_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("ready");
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand (return home)", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("close");
    task.add(std::move(stage));
  }

  return task;
}

}  // namespace orion_mtc
