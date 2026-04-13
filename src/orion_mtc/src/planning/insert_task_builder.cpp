/* InsertTaskBuilder：peg-in-hole 多段 LIN/Cartesian，可选倒角平面微动 */

#include "orion_mtc/planning/insert_task_builder.hpp"
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace mtc = moveit::task_constructor;

namespace orion_mtc
{

namespace
{

Eigen::Vector3d normalized_insert_axis_local(const std::vector<double>& axis_local_param)
{
  if (axis_local_param.size() != 3u)
  {
    return Eigen::Vector3d::UnitX();
  }
  const Eigen::Vector3d axis(axis_local_param[0], axis_local_param[1], axis_local_param[2]);
  const double n = axis.norm();
  if (!std::isfinite(axis.x()) || !std::isfinite(axis.y()) || !std::isfinite(axis.z()) || n < 1e-9)
  {
    return Eigen::Vector3d::UnitX();
  }
  return axis / n;
}

Eigen::Vector3d insert_axis_from_hole_pose(const geometry_msgs::msg::Pose& p,
                                           const std::vector<double>& axis_local_param)
{
  const Eigen::Vector3d axis_local = normalized_insert_axis_local(axis_local_param);
  const Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  Eigen::Vector3d v = q * axis_local;
  const double n = v.norm();
  if (n < 1e-9)
  {
    return axis_local;
  }
  return v / n;
}

void append_vector3_stamped(const rclcpp::Time& stamp, const std::string& frame,
                             const Eigen::Vector3d& dir, geometry_msgs::msg::Vector3Stamped& out)
{
  out.header.stamp = stamp;
  out.header.frame_id = frame;
  out.vector.x = dir.x();
  out.vector.y = dir.y();
  out.vector.z = dir.z();
}

}  // namespace

InsertTaskBuilder::InsertTaskBuilder(const rclcpp::Node::SharedPtr& node, const MTCConfig& config)
  : node_(node), config_(config)
{
}

InsertTaskBuildResult InsertTaskBuilder::buildTargetInsertTask(
    const geometry_msgs::msg::PoseStamped& target_pose) const
{
  InsertTaskBuildResult out;
  out.stage_names.clear();

  const std::string plan_frame = target_pose.header.frame_id;
  const PegInsertConfig& pi = config_.peg_insert;
  const double pre_offset = pi.pre_offset_m;
  const double insert_depth = pi.insert_depth_m;
  const double retreat_m = pi.retreat_m;
  const int axial_segments = std::max(1, pi.insert_axial_segments);
  const double axial_step = insert_depth / static_cast<double>(axial_segments);

  mtc::Task& task = out.task;
  task.stages()->setName("target insert");
  task.loadRobotModel(node_);
  const std::string arm_group_name = "arm";
  const std::string hand_group_name = "hand";
  const std::string hand_frame = "gripper_tcp";
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.stages()->properties().set("group", arm_group_name);
  task.stages()->properties().set("eef", hand_group_name);
  task.stages()->properties().set("ik_frame", hand_frame);

  task.add(std::make_unique<mtc::stages::CurrentState>("current"));
  out.stage_names.push_back("current");

  auto ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  ptp_planner->setPlannerId("PTP");
  auto lin_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  lin_planner->setPlannerId("LIN");
  lin_planner->setMaxVelocityScalingFactor(static_cast<double>(pi.lin_velocity_scaling));
  lin_planner->setMaxAccelerationScalingFactor(static_cast<double>(pi.lin_acceleration_scaling));
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(static_cast<double>(pi.cartesian_velocity_scaling));
  cartesian_planner->setMaxAccelerationScalingFactor(static_cast<double>(pi.cartesian_acceleration_scaling));
  cartesian_planner->setStepSize(static_cast<double>(pi.cartesian_step_size));
  cartesian_planner->setMinFraction(0.90);

  const Eigen::Vector3d axis_insert =
      insert_axis_from_hole_pose(target_pose.pose, pi.insert_axis_local_xyz);

  if (pi.go_ready_before_insert)
  {
    auto stage_ready = std::make_unique<mtc::stages::MoveTo>("move to ready (before insert)", ptp_planner);
    stage_ready->setGroup(arm_group_name);
    stage_ready->setGoal("ready");
    task.add(std::move(stage_ready));
    out.stage_names.push_back("move to ready (before insert)");
  }

  geometry_msgs::msg::PoseStamped axis_pre_pose = target_pose;
  axis_pre_pose.pose.position.x -= axis_insert.x() * pre_offset;
  axis_pre_pose.pose.position.y -= axis_insert.y() * pre_offset;
  axis_pre_pose.pose.position.z -= axis_insert.z() * pre_offset;
  const double front_waypoint_offset = std::max(0.0, pi.front_waypoint_offset_m);
  const double front_waypoint_base_x_offset = std::max(0.0, pi.front_waypoint_base_x_offset_m);
  const double pre_insert_base_x_offset = std::max(0.0, pi.pre_insert_base_x_offset_m);
  geometry_msgs::msg::PoseStamped pre_pose = axis_pre_pose;
  if (pi.pre_insert_use_base_x)
  {
    pre_pose = target_pose;
    pre_pose.pose.position.x -= pre_insert_base_x_offset;
  }
  geometry_msgs::msg::PoseStamped front_pose = pre_pose;
  if (pi.front_waypoint_use_base_x)
  {
    front_pose = target_pose;
    front_pose.pose.position.x -= front_waypoint_base_x_offset;
  }
  else
  {
    front_pose.pose.position.x -= axis_insert.x() * front_waypoint_offset;
    front_pose.pose.position.y -= axis_insert.y() * front_waypoint_offset;
    front_pose.pose.position.z -= axis_insert.z() * front_waypoint_offset;
  }
  const rclcpp::Time now = node_->now();
  pre_pose.header.stamp = now;
  axis_pre_pose.header.stamp = now;
  front_pose.header.stamp = now;

  const bool front_waypoint_enabled =
      pi.enable_front_waypoint
      && ((pi.front_waypoint_use_base_x && front_waypoint_base_x_offset > 1e-6)
          || (!pi.front_waypoint_use_base_x && front_waypoint_offset > 1e-6));
  if (front_waypoint_enabled)
  {
    auto move_front = std::make_unique<mtc::stages::MoveTo>("move to front-waypoint", ptp_planner);
    move_front->setGroup(arm_group_name);
    move_front->setGoal(front_pose);
    move_front->setIKFrame(hand_frame);
    task.add(std::move(move_front));
    out.stage_names.push_back("move to front-waypoint");

    if (pi.front_waypoint_use_base_x)
    {
      auto move_pre_align =
          std::make_unique<mtc::stages::MoveTo>("front-waypoint to pre-insert (align)", ptp_planner);
      move_pre_align->setGroup(arm_group_name);
      move_pre_align->setGoal(pre_pose);
      move_pre_align->setIKFrame(hand_frame);
      task.add(std::move(move_pre_align));
      out.stage_names.push_back("front-waypoint to pre-insert (align)");
    }
    else
    {
      geometry_msgs::msg::Vector3Stamped front_to_pre_axis;
      append_vector3_stamped(now, plan_frame, axis_insert, front_to_pre_axis);
      auto move_front_to_pre =
          std::make_unique<mtc::stages::MoveRelative>("front-waypoint to pre-insert", lin_planner);
      move_front_to_pre->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      move_front_to_pre->setIKFrame(hand_frame);
      move_front_to_pre->setDirection(front_to_pre_axis);
      move_front_to_pre->setMinMaxDistance(static_cast<float>(front_waypoint_offset),
                                           static_cast<float>(front_waypoint_offset));
      task.add(std::move(move_front_to_pre));
      out.stage_names.push_back("front-waypoint to pre-insert");
    }
  }
  else
  {
    auto move_pre = std::make_unique<mtc::stages::MoveTo>("move to pre-insert", ptp_planner);
    move_pre->setGroup(arm_group_name);
    move_pre->setGoal(pre_pose);
    move_pre->setIKFrame(hand_frame);
    task.add(std::move(move_pre));
    out.stage_names.push_back("move to pre-insert");
  }

  geometry_msgs::msg::Vector3Stamped axis_msg;
  append_vector3_stamped(now, plan_frame, axis_insert, axis_msg);
  auto descend_to_slot = std::make_unique<mtc::stages::MoveRelative>("insert approach", cartesian_planner);
  descend_to_slot->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  descend_to_slot->setIKFrame(hand_frame);
  descend_to_slot->setDirection(axis_msg);
  descend_to_slot->setMinMaxDistance(static_cast<float>(pre_offset), static_cast<float>(pre_offset));
  task.add(std::move(descend_to_slot));
  out.stage_names.push_back("insert approach");

  if (pi.enable_chamfer_plane_search && pi.chamfer_plane_delta_m > 1e-6)
  {
    const double delta = pi.chamfer_plane_delta_m;
    Eigen::Vector3d ref = Eigen::Vector3d::UnitY();
    if (std::abs(axis_insert.dot(ref)) > 0.9)
    {
      ref = Eigen::Vector3d::UnitX();
    }
    Eigen::Vector3d u = axis_insert.cross(ref).normalized();
    Eigen::Vector3d v = axis_insert.cross(u).normalized();

    const char* chamfer_names[4] = { "insert chamfer +u", "insert chamfer -u", "insert chamfer +v",
                                     "insert chamfer -v" };
    const Eigen::Vector3d dirs[4] = { u, -u, v, -v };
    for (int i = 0; i < 4; ++i)
    {
      geometry_msgs::msg::Vector3Stamped d_msg;
      append_vector3_stamped(node_->now(), plan_frame, dirs[i], d_msg);
      auto wiggle =
          std::make_unique<mtc::stages::MoveRelative>(std::string(chamfer_names[i]), lin_planner);
      wiggle->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      wiggle->setIKFrame(hand_frame);
      wiggle->setDirection(d_msg);
      wiggle->setMinMaxDistance(static_cast<float>(delta), static_cast<float>(delta));
      task.add(std::move(wiggle));
      out.stage_names.push_back(chamfer_names[i]);
    }
  }

  for (int s = 0; s < axial_segments; ++s)
  {
    const std::string seg_name =
        (axial_segments > 1) ? ("insert descend segment " + std::to_string(s + 1)) : "insert descend";
    auto insert_seg = std::make_unique<mtc::stages::MoveRelative>(seg_name, cartesian_planner);
    insert_seg->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    insert_seg->setIKFrame(hand_frame);
    insert_seg->setDirection(axis_msg);
    insert_seg->setMinMaxDistance(static_cast<float>(axial_step), static_cast<float>(axial_step));
    task.add(std::move(insert_seg));
    out.stage_names.push_back(seg_name);
  }

  auto stage_open = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open->setGroup(hand_group_name);
  stage_open->setGoal("open");
  task.add(std::move(stage_open));
  out.stage_names.push_back("open hand");

  geometry_msgs::msg::Vector3Stamped retreat_axis;
  append_vector3_stamped(node_->now(), plan_frame, -axis_insert, retreat_axis);
  const int retreat_segments = 3;
  const double retreat_step = retreat_m / static_cast<double>(retreat_segments);
  for (int s = 0; s < retreat_segments; ++s)
  {
    const std::string retreat_name =
        (retreat_segments > 1) ? ("retreat up segment " + std::to_string(s + 1)) : "retreat up";
    auto retreat_up = std::make_unique<mtc::stages::MoveRelative>(retreat_name, cartesian_planner);
    retreat_up->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    retreat_up->setIKFrame(hand_frame);
    retreat_up->setDirection(retreat_axis);
    retreat_up->setMinMaxDistance(static_cast<float>(retreat_step), static_cast<float>(retreat_step));
    task.add(std::move(retreat_up));
    out.stage_names.push_back(retreat_name);
  }

  return out;
}

}  // namespace orion_mtc
