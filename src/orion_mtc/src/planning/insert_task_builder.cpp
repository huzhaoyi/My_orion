/* InsertTaskBuilder：peg-in-hole 多段 LIN/Cartesian，可选倒角平面微动 */

#include "orion_mtc/planning/insert_task_builder.hpp"
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace mtc = moveit::task_constructor;

namespace orion_mtc
{

namespace
{

double deg_to_rad(double deg)
{
  return deg * M_PI / 180.0;
}

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

Eigen::Vector3d peg_rod_axis_tcp_unit_from_param(const std::vector<double>& peg_rod_axis_tcp_xyz)
{
  Eigen::Vector3d rod_tcp(0.0, 1.0, 0.0);
  if (peg_rod_axis_tcp_xyz.size() == 3u)
  {
    rod_tcp << peg_rod_axis_tcp_xyz[0], peg_rod_axis_tcp_xyz[1], peg_rod_axis_tcp_xyz[2];
  }
  const double rod_n = rod_tcp.norm();
  if (!std::isfinite(rod_tcp.x()) || !std::isfinite(rod_tcp.y()) || !std::isfinite(rod_tcp.z()) || rod_n < 1e-9)
  {
    return Eigen::Vector3d(0.0, 1.0, 0.0);
  }
  return rod_tcp / rod_n;
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

void apply_insert_hole_pose_adjustments(geometry_msgs::msg::PoseStamped& target_pose_adjusted,
                                        const rclcpp::Node::SharedPtr& node,
                                        const PegInsertConfig& pi)
{
  const double tool_roll_rad = pi.tool_roll_about_insert_axis_deg * M_PI / 180.0;
  if (std::abs(tool_roll_rad) > 1e-9)
  {
    const Eigen::Vector3d axis_insert_nominal =
        insert_axis_from_hole_pose(target_pose_adjusted.pose, pi.insert_axis_local_xyz);
    const Eigen::Quaterniond q_nominal(
        target_pose_adjusted.pose.orientation.w,
        target_pose_adjusted.pose.orientation.x,
        target_pose_adjusted.pose.orientation.y,
        target_pose_adjusted.pose.orientation.z);
    const Eigen::Quaterniond q_roll(Eigen::AngleAxisd(tool_roll_rad, axis_insert_nominal));
    const Eigen::Quaterniond q_adjusted = (q_roll * q_nominal).normalized();
    target_pose_adjusted.pose.orientation.x = q_adjusted.x();
    target_pose_adjusted.pose.orientation.y = q_adjusted.y();
    target_pose_adjusted.pose.orientation.z = q_adjusted.z();
    target_pose_adjusted.pose.orientation.w = q_adjusted.w();
  }
  if (pi.tool_rpy_offset_deg.size() == 3u)
  {
    const double roll_rad = deg_to_rad(pi.tool_rpy_offset_deg[0]);
    const double pitch_rad = deg_to_rad(pi.tool_rpy_offset_deg[1]);
    const double yaw_rad = deg_to_rad(pi.tool_rpy_offset_deg[2]);
    if (std::abs(roll_rad) > 1e-9 || std::abs(pitch_rad) > 1e-9 || std::abs(yaw_rad) > 1e-9)
    {
      const Eigen::Quaterniond q_nominal(
          target_pose_adjusted.pose.orientation.w,
          target_pose_adjusted.pose.orientation.x,
          target_pose_adjusted.pose.orientation.y,
          target_pose_adjusted.pose.orientation.z);
      const Eigen::Quaterniond q_offset =
          Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX());
      const Eigen::Quaterniond q_adjusted = (q_nominal * q_offset).normalized();
      target_pose_adjusted.pose.orientation.x = q_adjusted.x();
      target_pose_adjusted.pose.orientation.y = q_adjusted.y();
      target_pose_adjusted.pose.orientation.z = q_adjusted.z();
      target_pose_adjusted.pose.orientation.w = q_adjusted.w();
    }
  }
  if (pi.auto_flip_rod_to_insert)
  {
    const Eigen::Vector3d rod_tcp = peg_rod_axis_tcp_unit_from_param(pi.peg_rod_axis_tcp_xyz);
    const Eigen::Quaterniond q_now(target_pose_adjusted.pose.orientation.w,
                                   target_pose_adjusted.pose.orientation.x,
                                   target_pose_adjusted.pose.orientation.y,
                                   target_pose_adjusted.pose.orientation.z);
    const Eigen::Vector3d rod_w = (q_now * rod_tcp).normalized();
    const Eigen::Vector3d axis_ins =
        insert_axis_from_hole_pose(target_pose_adjusted.pose, pi.insert_axis_local_xyz);
    const double align_dot = rod_w.dot(axis_ins);
    if (align_dot < -1e-3)
    {
      Eigen::Vector3d flip_axis = rod_w.cross(axis_ins);
      if (flip_axis.norm() < 0.12)
      {
        flip_axis = rod_w.cross(Eigen::Vector3d::UnitZ());
      }
      if (flip_axis.norm() < 0.12)
      {
        flip_axis = rod_w.cross(Eigen::Vector3d::UnitX());
      }
      const double fn = flip_axis.norm();
      if (fn >= 1e-9)
      {
        flip_axis /= fn;
        const Eigen::Quaterniond q_rot(Eigen::AngleAxisd(M_PI, flip_axis));
        const Eigen::Quaterniond q_twist = (q_rot * q_now).normalized();
        target_pose_adjusted.pose.orientation.x = q_twist.x();
        target_pose_adjusted.pose.orientation.y = q_twist.y();
        target_pose_adjusted.pose.orientation.z = q_twist.z();
        target_pose_adjusted.pose.orientation.w = q_twist.w();
        RCLCPP_INFO(node->get_logger(),
                    "insert_task_builder: auto_flip_rod_to_insert align_dot=%.4f flip_axis=(%.3f,%.3f,%.3f)",
                    align_dot, flip_axis.x(), flip_axis.y(), flip_axis.z());
      }
    }
    else
    {
      RCLCPP_INFO(node->get_logger(),
                  "insert_task_builder: auto_flip_rod_to_insert skipped align_dot=%.4f",
                  align_dot);
    }
  }
  if (pi.swap_tip_handle_180)
  {
    const Eigen::Vector3d rod_tcp = peg_rod_axis_tcp_unit_from_param(pi.peg_rod_axis_tcp_xyz);
    const Eigen::Quaterniond q_now(target_pose_adjusted.pose.orientation.w,
                                     target_pose_adjusted.pose.orientation.x,
                                     target_pose_adjusted.pose.orientation.y,
                                     target_pose_adjusted.pose.orientation.z);
    const Eigen::Vector3d axis_world =
        insert_axis_from_hole_pose(target_pose_adjusted.pose, pi.insert_axis_local_xyz);
    const Eigen::Matrix3d R = q_now.toRotationMatrix();
    Eigen::Vector3d axis_tcp = R.transpose() * axis_world;
    const double axis_tcp_n = axis_tcp.norm();
    if (axis_tcp_n > 1e-9)
    {
      axis_tcp /= axis_tcp_n;
    }
    else
    {
      axis_tcp = Eigen::Vector3d::UnitZ();
    }
    Eigen::Vector3d hinge_tcp = rod_tcp.cross(axis_tcp);
    if (hinge_tcp.norm() < 0.12)
    {
      hinge_tcp = axis_tcp.cross(rod_tcp);
    }
    if (hinge_tcp.norm() < 0.12)
    {
      hinge_tcp = rod_tcp.cross(Eigen::Vector3d::UnitX());
    }
    if (hinge_tcp.norm() < 0.12)
    {
      hinge_tcp = rod_tcp.cross(Eigen::Vector3d::UnitY());
    }
    if (hinge_tcp.norm() < 1e-6)
    {
      RCLCPP_WARN(node->get_logger(),
                  "insert_task_builder: swap_tip_handle_180 skipped (no hinge perpendicular to rod_tcp)");
    }
    else
    {
      hinge_tcp.normalize();
      const Eigen::Quaterniond q_h(Eigen::AngleAxisd(M_PI, hinge_tcp));
      const Eigen::Quaterniond q_out = (q_now * q_h).normalized();
      target_pose_adjusted.pose.orientation.x = q_out.x();
      target_pose_adjusted.pose.orientation.y = q_out.y();
      target_pose_adjusted.pose.orientation.z = q_out.z();
      target_pose_adjusted.pose.orientation.w = q_out.w();
      RCLCPP_INFO(node->get_logger(),
                  "insert_task_builder: swap_tip_handle_180 applied hinge_tcp=(%.3f,%.3f,%.3f) "
                  "axis_tcp=(%.3f,%.3f,%.3f) rod_tcp=(%.3f,%.3f,%.3f)",
                  hinge_tcp.x(), hinge_tcp.y(), hinge_tcp.z(), axis_tcp.x(), axis_tcp.y(), axis_tcp.z(),
                  rod_tcp.x(), rod_tcp.y(), rod_tcp.z());
    }
  }
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

  const PegInsertConfig& pi = config_.peg_insert;
  geometry_msgs::msg::PoseStamped target_pose_adjusted = target_pose;
  apply_insert_hole_pose_adjustments(target_pose_adjusted, node_, pi);
  const std::string plan_frame = target_pose_adjusted.header.frame_id;
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
  // 使用 peg_tip（被持物插头尖端参考帧）作为 IK frame，而非 gripper_tcp 原点。
  // peg_tip 相对 gripper_tcp 的偏移见 URDF joint_gripper_tcp_peg_tip（|y|≈0.08m 沿 peg 长轴，z≈0.035m）。
  // 这样 MTC 将插头尖端对准孔位，而非将 gripper_tcp 原点对准孔位，解决径向错位导致的插不进问题。
  const std::string hand_frame = "peg_tip";
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

  Eigen::Vector3d axis_insert =
      insert_axis_from_hole_pose(target_pose_adjusted.pose, pi.insert_axis_local_xyz);
  double axis_sign = pi.insert_motion_axis_sign;
  if (!std::isfinite(axis_sign) || std::abs(axis_sign) < 1e-9)
  {
    axis_sign = 1.0;
  }
  axis_insert *= axis_sign;
  {
    const double nn = axis_insert.norm();
    if (nn > 1e-9)
    {
      axis_insert /= nn;
    }
  }

  if (pi.go_ready_before_insert)
  {
    auto stage_ready = std::make_unique<mtc::stages::MoveTo>("move to ready (before insert)", ptp_planner);
    stage_ready->setGroup(arm_group_name);
    stage_ready->setGoal("ready");
    task.add(std::move(stage_ready));
    out.stage_names.push_back("move to ready (before insert)");
  }

  geometry_msgs::msg::PoseStamped axis_pre_pose = target_pose_adjusted;
  axis_pre_pose.pose.position.x -= axis_insert.x() * pre_offset;
  axis_pre_pose.pose.position.y -= axis_insert.y() * pre_offset;
  axis_pre_pose.pose.position.z -= axis_insert.z() * pre_offset;
  const double front_waypoint_offset = std::max(0.0, pi.front_waypoint_offset_m);
  const double front_waypoint_base_x_offset = std::max(0.0, pi.front_waypoint_base_x_offset_m);
  const double pre_insert_base_x_offset = std::max(0.0, pi.pre_insert_base_x_offset_m);
  geometry_msgs::msg::PoseStamped pre_pose = axis_pre_pose;
  if (pi.pre_insert_use_base_x)
  {
    pre_pose = target_pose_adjusted;
    pre_pose.pose.position.x -= pre_insert_base_x_offset;
  }
  geometry_msgs::msg::PoseStamped front_pose = pre_pose;
  if (pi.front_waypoint_use_base_x)
  {
    front_pose = target_pose_adjusted;
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

  if (retreat_m > 1e-6)
  {
    // 抬升脱离沿“当前插入轴反方向”退离，避免候选姿态变化时固定 +Z 引发擦碰。
    Eigen::Vector3d retreat_axis = -axis_insert;
    if (!std::isfinite(retreat_axis.x()) || !std::isfinite(retreat_axis.y()) || !std::isfinite(retreat_axis.z())
        || retreat_axis.norm() < 1e-9)
    {
      retreat_axis = Eigen::Vector3d::UnitZ();
    }
    else
    {
      retreat_axis.normalize();
    }
    geometry_msgs::msg::Vector3Stamped lift_axis;
    append_vector3_stamped(node_->now(), plan_frame, retreat_axis, lift_axis);
    const int lift_segments = 3;
    const double lift_step = retreat_m / static_cast<double>(lift_segments);
    for (int s = 0; s < lift_segments; ++s)
    {
      const std::string lift_name =
          (lift_segments > 1) ? ("lift clear segment " + std::to_string(s + 1)) : "lift clear";
      auto lift_clear = std::make_unique<mtc::stages::MoveRelative>(lift_name, cartesian_planner);
      lift_clear->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      lift_clear->setIKFrame(hand_frame);
      lift_clear->setDirection(lift_axis);
      lift_clear->setMinMaxDistance(static_cast<float>(lift_step), static_cast<float>(lift_step));
      task.add(std::move(lift_clear));
      out.stage_names.push_back(lift_name);
    }
  }

  // 释放后先回到插入前安全路点，再回 ready，避免从孔位附近直接 PTP 扫过 target。
  auto stage_post_release_pre =
      std::make_unique<mtc::stages::MoveTo>("move to pre-insert (post release)", ptp_planner);
  stage_post_release_pre->setGroup(arm_group_name);
  stage_post_release_pre->setGoal(pre_pose);
  stage_post_release_pre->setIKFrame(hand_frame);
  task.add(std::move(stage_post_release_pre));
  out.stage_names.push_back("move to pre-insert (post release)");

  if (front_waypoint_enabled)
  {
    auto stage_post_release_front =
        std::make_unique<mtc::stages::MoveTo>("move to front-waypoint (post release)", ptp_planner);
    stage_post_release_front->setGroup(arm_group_name);
    stage_post_release_front->setGoal(front_pose);
    stage_post_release_front->setIKFrame(hand_frame);
    task.add(std::move(stage_post_release_front));
    out.stage_names.push_back("move to front-waypoint (post release)");
  }

  auto stage_ready_after_release = std::make_unique<mtc::stages::MoveTo>("move to ready (after release)", ptp_planner);
  stage_ready_after_release->setGroup(arm_group_name);
  stage_ready_after_release->setGoal("ready");
  task.add(std::move(stage_ready_after_release));
  out.stage_names.push_back("move to ready (after release)");

  return out;
}

bool InsertTaskBuilder::planArmLiftRetreatSubTrajectories(
    const geometry_msgs::msg::PoseStamped& hole_pose_base,
    std::vector<moveit_task_constructor_msgs::msg::SubTrajectory>& out_arm_segments) const
{
  out_arm_segments.clear();
  const PegInsertConfig& pi = config_.peg_insert;
  const double retreat_m = pi.retreat_m;
  if (retreat_m <= 1e-6)
  {
    RCLCPP_WARN(node_->get_logger(),
                "insert_task_builder: planArmLiftRetreatSubTrajectories skipped (retreat_m<=0)");
    return false;
  }

  geometry_msgs::msg::PoseStamped adjusted = hole_pose_base;
  apply_insert_hole_pose_adjustments(adjusted, node_, pi);

  Eigen::Vector3d axis_insert =
      insert_axis_from_hole_pose(adjusted.pose, pi.insert_axis_local_xyz);
  double axis_sign = pi.insert_motion_axis_sign;
  if (!std::isfinite(axis_sign) || std::abs(axis_sign) < 1e-9)
  {
    axis_sign = 1.0;
  }
  axis_insert *= axis_sign;
  {
    const double nn = axis_insert.norm();
    if (nn > 1e-9)
    {
      axis_insert /= nn;
    }
  }

  Eigen::Vector3d retreat_axis = -axis_insert;
  if (!std::isfinite(retreat_axis.x()) || !std::isfinite(retreat_axis.y()) || !std::isfinite(retreat_axis.z())
      || retreat_axis.norm() < 1e-9)
  {
    retreat_axis = Eigen::Vector3d::UnitZ();
  }
  else
  {
    retreat_axis.normalize();
  }

  const std::string plan_frame = adjusted.header.frame_id;
  const rclcpp::Time now = node_->now();
  geometry_msgs::msg::Vector3Stamped lift_axis;
  append_vector3_stamped(now, plan_frame, retreat_axis, lift_axis);

  mtc::Task task;
  task.stages()->setName("insert lift replan");
  task.loadRobotModel(node_);
  const std::string arm_group_name = "arm";
  const std::string hand_group_name = "hand";
  const std::string hand_frame = "peg_tip";
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.stages()->properties().set("group", arm_group_name);
  task.stages()->properties().set("eef", hand_group_name);
  task.stages()->properties().set("ik_frame", hand_frame);

  task.add(std::make_unique<mtc::stages::CurrentState>("current"));

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(static_cast<double>(pi.cartesian_velocity_scaling));
  cartesian_planner->setMaxAccelerationScalingFactor(static_cast<double>(pi.cartesian_acceleration_scaling));
  cartesian_planner->setStepSize(static_cast<double>(pi.cartesian_step_size));
  cartesian_planner->setMinFraction(0.90);

  constexpr int k_lift_segments = 3;
  const double lift_step = retreat_m / static_cast<double>(k_lift_segments);
  for (int s = 0; s < k_lift_segments; ++s)
  {
    const std::string lift_name =
        (k_lift_segments > 1) ? ("lift clear segment " + std::to_string(s + 1)) : "lift clear";
    auto lift_clear = std::make_unique<mtc::stages::MoveRelative>(lift_name, cartesian_planner);
    lift_clear->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    lift_clear->setIKFrame(hand_frame);
    lift_clear->setDirection(lift_axis);
    lift_clear->setMinMaxDistance(static_cast<float>(lift_step), static_cast<float>(lift_step));
    task.add(std::move(lift_clear));
  }

  try
  {
    task.init();
    task.enableIntrospection(true);
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "insert planArmLiftRetreatSubTrajectories init failed: " << e);
    return false;
  }

  moveit::core::MoveItErrorCode plan_result = task.plan(5);
  if (!plan_result || task.solutions().empty())
  {
    std::ostringstream os;
    task.explainFailure(os);
    RCLCPP_WARN_STREAM(node_->get_logger(),
                       "insert planArmLiftRetreatSubTrajectories plan failed: " << os.str());
    return false;
  }

  moveit_task_constructor_msgs::msg::Solution sol_msg;
  task.solutions().front()->toMsg(sol_msg, &task.introspection());

  if (sol_msg.sub_trajectory.size() == static_cast<size_t>(k_lift_segments))
  {
    out_arm_segments = sol_msg.sub_trajectory;
    RCLCPP_INFO(node_->get_logger(),
                "insert_task_builder: planArmLiftRetreatSubTrajectories ok (%zu subs, no current state segment)",
                out_arm_segments.size());
    return true;
  }
  if (sol_msg.sub_trajectory.size() == static_cast<size_t>(k_lift_segments) + 1u)
  {
    for (size_t j = 1; j < sol_msg.sub_trajectory.size(); ++j)
    {
      out_arm_segments.push_back(sol_msg.sub_trajectory[j]);
    }
    const bool ok = out_arm_segments.size() == static_cast<size_t>(k_lift_segments);
    if (ok)
    {
      RCLCPP_INFO(node_->get_logger(),
                  "insert_task_builder: planArmLiftRetreatSubTrajectories ok (%zu lift subs after current)",
                  out_arm_segments.size());
    }
    return ok;
  }

  RCLCPP_WARN(node_->get_logger(),
              "insert planArmLiftRetreatSubTrajectories: unexpected sub_trajectory count %zu",
              sol_msg.sub_trajectory.size());
  return false;
}

bool InsertTaskBuilder::planPostReleaseMoveToPreInsertSubTrajectory(
    const geometry_msgs::msg::PoseStamped& hole_pose_base,
    moveit_task_constructor_msgs::msg::SubTrajectory& out_arm_sub) const
{
  const PegInsertConfig& pi = config_.peg_insert;
  geometry_msgs::msg::PoseStamped adjusted = hole_pose_base;
  apply_insert_hole_pose_adjustments(adjusted, node_, pi);

  Eigen::Vector3d axis_insert =
      insert_axis_from_hole_pose(adjusted.pose, pi.insert_axis_local_xyz);
  double axis_sign = pi.insert_motion_axis_sign;
  if (!std::isfinite(axis_sign) || std::abs(axis_sign) < 1e-9)
  {
    axis_sign = 1.0;
  }
  axis_insert *= axis_sign;
  {
    const double nn = axis_insert.norm();
    if (nn > 1e-9)
    {
      axis_insert /= nn;
    }
  }

  const double pre_offset = pi.pre_offset_m;
  geometry_msgs::msg::PoseStamped axis_pre_pose = adjusted;
  axis_pre_pose.pose.position.x -= axis_insert.x() * pre_offset;
  axis_pre_pose.pose.position.y -= axis_insert.y() * pre_offset;
  axis_pre_pose.pose.position.z -= axis_insert.z() * pre_offset;
  const double pre_insert_base_x_offset = std::max(0.0, pi.pre_insert_base_x_offset_m);
  geometry_msgs::msg::PoseStamped pre_pose = axis_pre_pose;
  if (pi.pre_insert_use_base_x)
  {
    pre_pose = adjusted;
    pre_pose.pose.position.x -= pre_insert_base_x_offset;
  }
  const rclcpp::Time now = node_->now();
  pre_pose.header.stamp = now;
  pre_pose.header.frame_id = adjusted.header.frame_id;

  mtc::Task task;
  task.stages()->setName("insert post-release pre replan");
  task.loadRobotModel(node_);
  const std::string arm_group_name = "arm";
  const std::string hand_group_name = "hand";
  const std::string hand_frame = "peg_tip";
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.stages()->properties().set("group", arm_group_name);
  task.stages()->properties().set("eef", hand_group_name);
  task.stages()->properties().set("ik_frame", hand_frame);

  task.add(std::make_unique<mtc::stages::CurrentState>("current"));

  auto ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  ptp_planner->setPlannerId("PTP");
  ptp_planner->setMaxVelocityScalingFactor(static_cast<double>(pi.lin_velocity_scaling));
  ptp_planner->setMaxAccelerationScalingFactor(static_cast<double>(pi.lin_acceleration_scaling));
  auto stage_post =
      std::make_unique<mtc::stages::MoveTo>("move to pre-insert (post release)", ptp_planner);
  stage_post->setGroup(arm_group_name);
  stage_post->setGoal(pre_pose);
  stage_post->setIKFrame(hand_frame);
  task.add(std::move(stage_post));

  try
  {
    task.init();
    task.enableIntrospection(true);
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "insert planPostReleaseMoveToPreInsert init failed: " << e);
    return false;
  }

  moveit::core::MoveItErrorCode plan_result = task.plan(5);
  if (!plan_result || task.solutions().empty())
  {
    std::ostringstream os;
    task.explainFailure(os);
    RCLCPP_WARN_STREAM(node_->get_logger(),
                       "insert planPostReleaseMoveToPreInsert plan failed: " << os.str());
    return false;
  }

  moveit_task_constructor_msgs::msg::Solution sol_msg;
  task.solutions().front()->toMsg(sol_msg, &task.introspection());
  if (sol_msg.sub_trajectory.size() == 2u)
  {
    out_arm_sub = sol_msg.sub_trajectory[1];
    RCLCPP_INFO(node_->get_logger(),
                "insert_task_builder: planPostReleaseMoveToPreInsert ok (sub after current)");
    return true;
  }
  if (sol_msg.sub_trajectory.size() == 1u)
  {
    out_arm_sub = sol_msg.sub_trajectory[0];
    RCLCPP_INFO(node_->get_logger(),
                "insert_task_builder: planPostReleaseMoveToPreInsert ok (single sub)");
    return true;
  }

  RCLCPP_WARN(node_->get_logger(),
              "insert planPostReleaseMoveToPreInsert: unexpected sub_trajectory count %zu",
              sol_msg.sub_trajectory.size());
  return false;
}

}  // namespace orion_mtc
