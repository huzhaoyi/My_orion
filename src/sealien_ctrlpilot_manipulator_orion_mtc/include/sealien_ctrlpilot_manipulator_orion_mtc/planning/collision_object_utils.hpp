/* 纯几何/碰撞体工具：composePose、makeTargetCollisionObject，非业务逻辑 */

#ifndef ORION_MTC_PLANNING_COLLISION_OBJECT_UTILS_HPP
#define ORION_MTC_PLANNING_COLLISION_OBJECT_UTILS_HPP

#include "sealien_ctrlpilot_manipulator_orion_mtc/planning/cable_segments.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <array>
#include <cstdint>
#include <vector>

namespace sealien_ctrlpilot_manipulator_orion_mtc
{

/* 局部 pose 在整体 pose 下变换：result = base * local（世界系下的 local） */
geometry_msgs::msg::Pose composePose(const geometry_msgs::msg::Pose& base,
                                      const geometry_msgs::msg::Pose& local);

/* 统一生成 target 组合碰撞体：杆 + 把手框，primitive_poses 为世界系（arm_base_link） */
moveit_msgs::msg::CollisionObject makeTargetCollisionObject(const std::string& object_id,
                                                             const geometry_msgs::msg::Pose& object_pose,
                                                             uint8_t operation);

/*
 * TargetSensor peg：package://sealien_ctrlpilot_manipulator_orion_description/meshes/stl/target_new.stl，mesh 局部旋转与前端 RobotScene.js
 *（bbox 对齐长轴到 +Y + 绕局部 X 躺平）一致；object_pose 为感知物体位姿（arm_base_link）。
 */
moveit_msgs::msg::CollisionObject makeTargetSensorPegCollisionObject(const std::string& object_id,
                                                                      const std::string& frame_id,
                                                                      const geometry_msgs::msg::Pose& object_pose,
                                                                      uint8_t operation);

/* 单段缆绳短圆柱碰撞体：局部 Z 对齐 segment.axis，中心在 segment.center */
moveit_msgs::msg::CollisionObject makeSegmentCollisionObject(const CableSegment& segment,
                                                              const std::string& frame_id,
                                                              uint8_t operation);

/*
 * 孔位旁面板：四角点（frame_id 下坐标，展平 x,y,z×4）经 unit_scale 缩放到米后求 AABB，
 * 各轴尺寸小于 wall_thickness_m 时钳到薄盒厚度；整体再扩 aabb_margin_m（各向半宽）。
 */
moveit_msgs::msg::CollisionObject makePanelBoxCollisionObject(const std::string& object_id,
                                                               const std::string& frame_id,
                                                               const std::vector<double>& corners_xyz,
                                                               double unit_scale,
                                                               double wall_thickness_m,
                                                               double aabb_margin_m,
                                                               uint8_t operation);

/*
 * 持物杆体（peg）附着碰撞体：沿 rod_axis_link（attach link 局部系单位向量）的圆柱，
 * 杆尖位于 attach link 原点（peg_tip），杆身朝 +rod_axis 方向延伸 length；用于插孔时 MoveIt
 * 对 peg↔面板做几何避障。frame_id 应为附着 link 名（如 "peg_tip"）。
 */
moveit_msgs::msg::CollisionObject makeHeldPegCollisionObject(const std::string& object_id,
                                                             const std::string& frame_id,
                                                             const std::vector<double>& rod_axis_link,
                                                             double length_m,
                                                             double radius_m,
                                                             uint8_t operation);

/*
 * 「带孔门板」碰撞体：在过孔心、法向为 insert_axis 的平面上，由 4 个 box 围出 2*gap_half 见方的缺口。
 * 对准的 peg 从缺口穿过（无碰撞），对不准的 peg 撞到板框（碰撞）。所有 box 以世界系（frame_id）primitive_poses 表达。
 * hole_center/insert_axis 均为 frame_id 下坐标；plane_offset 沿 -insert_axis 把板面前移（板在孔心前方的距离）。
 */
moveit_msgs::msg::CollisionObject makePanelWithHoleCollisionObject(const std::string& object_id,
                                                                   const std::string& frame_id,
                                                                   const std::array<double, 3>& hole_center,
                                                                   const std::array<double, 3>& insert_axis,
                                                                   double gap_half_m,
                                                                   double panel_half_size_m,
                                                                   double thickness_m,
                                                                   double plane_offset_m,
                                                                   uint8_t operation);

}  // namespace sealien_ctrlpilot_manipulator_orion_mtc

#endif  // ORION_MTC_PLANNING_COLLISION_OBJECT_UTILS_HPP
