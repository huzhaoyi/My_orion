#include "orion_mtc/planning/place_task_builder.hpp"
#include "orion_mtc/core/constants.hpp"
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mtc = moveit::task_constructor;

namespace orion_mtc
{

PlaceTaskBuilder::PlaceTaskBuilder(const rclcpp::Node::SharedPtr& node, const MTCConfig& config)
  : node_(node), config_(config)
{
}

mtc::Task PlaceTaskBuilder::build(double place_x, double place_y, double place_z,
                                  double place_qx, double place_qy, double place_qz, double place_qw,
                                  const HeldObjectContext& held)
{
  if (!held.valid)
  {
    mtc::Task empty;
    empty.loadRobotModel(node_);
    return empty;
  }
  std::string scene_attach_id = held.scene_attach_id.empty() ? "object" : held.scene_attach_id;

  Eigen::Isometry3d T_base_obj_target = Eigen::Isometry3d::Identity();
  T_base_obj_target.translate(Eigen::Vector3d(place_x, place_y, place_z));
  T_base_obj_target.rotate(Eigen::Quaterniond(place_qw, place_qx, place_qy, place_qz));
  Eigen::Isometry3d T_base_tcp_target = T_base_obj_target * held.tcp_to_object.inverse();

  const double lower_max = config_.lower_to_place_max_dist;
  Eigen::Vector3d pre_place_trans = T_base_tcp_target.translation();
  pre_place_trans.z() += lower_max;

  geometry_msgs::msg::Pose pre_place_pose;
  pre_place_pose.position.x = pre_place_trans.x();
  pre_place_pose.position.y = pre_place_trans.y();
  pre_place_pose.position.z = pre_place_trans.z();
  Eigen::Quaterniond q(T_base_tcp_target.rotation());
  pre_place_pose.orientation.x = q.x();
  pre_place_pose.orientation.y = q.y();
  pre_place_pose.orientation.z = q.z();
  pre_place_pose.orientation.w = q.w();

  mtc::Task task;
  task.stages()->setName("orion place");
  task.loadRobotModel(node_);
  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "Link6";
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  task.add(std::make_unique<mtc::stages::CurrentState>("current"));

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    stage->allowCollisions(scene_attach_id, OBJECT_GRASP_ALLOWED_LINKS, true);
    task.add(std::move(stage));
  }

  auto ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  ptp_planner->setPlannerId("PTP");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  geometry_msgs::msg::PoseStamped pre_place_stamped;
  pre_place_stamped.header.frame_id = "base_link";
  pre_place_stamped.pose = pre_place_pose;
  auto stage_pre = std::make_unique<mtc::stages::MoveTo>("move to pre-place", ptp_planner);
  stage_pre->setGroup(arm_group_name);
  stage_pre->setGoal(pre_place_stamped);
  stage_pre->setIKFrame(hand_frame);
  task.add(std::move(stage_pre));

  auto place = std::make_unique<mtc::SerialContainer>("place object");
  place->properties().set("eef", hand_group_name);
  place->properties().set("group", arm_group_name);
  place->properties().set("ik_frame", hand_frame);

  auto stage_lower = std::make_unique<mtc::stages::MoveRelative>("lower to place", cartesian_planner);
  stage_lower->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_lower->setMinMaxDistance(static_cast<float>(config_.lower_to_place_min_dist),
                                static_cast<float>(config_.lower_to_place_max_dist));
  stage_lower->setIKFrame(hand_frame);
  stage_lower->properties().set("marker_ns", "lower_place");
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

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage->detachObject(scene_attach_id, hand_frame);
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

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
    stage->allowCollisions(scene_attach_id, OBJECT_GRASP_ALLOWED_LINKS, false);
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
