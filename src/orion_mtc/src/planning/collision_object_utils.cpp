/* collision_object_utils：位姿链乘与目标/缆段 MoveIt CollisionObject 构建 */

#include "orion_mtc/planning/collision_object_utils.hpp"
#include "orion_mtc/decision/cylinder_side_grasp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <string>

namespace orion_mtc
{

namespace
{

/*
 * 读二进制 STL 的轴对齐包围盒边长；与 RobotScene.js 中 geometry.boundingBox.getSize 一致。
 */
bool read_stl_binary_aabb(const std::string& path, double& ex, double& ey, double& ez)
{
  std::ifstream file(path, std::ios::binary);
  if (!file)
  {
    return false;
  }
  file.seekg(0, std::ios::end);
  const std::streamoff file_len = file.tellg();
  if (file_len < 84)
  {
    return false;
  }
  file.seekg(0);
  char header[80];
  file.read(header, 80);
  if (!file)
  {
    return false;
  }
  uint32_t tri_count = 0;
  file.read(reinterpret_cast<char*>(&tri_count), sizeof(tri_count));
  if (!file)
  {
    return false;
  }
  const std::streamoff expected = 84 + static_cast<std::streamoff>(tri_count) * 50;
  if (tri_count == 0 || expected != file_len)
  {
    return false;
  }
  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();
  float max_z = std::numeric_limits<float>::lowest();
  for (uint32_t t = 0; t < tri_count; ++t)
  {
    float tri[12];
    file.read(reinterpret_cast<char*>(tri), sizeof(tri));
    if (!file)
    {
      return false;
    }
    for (int v = 0; v < 3; ++v)
    {
      const float vx = tri[3 + v * 3 + 0];
      const float vy = tri[3 + v * 3 + 1];
      const float vz = tri[3 + v * 3 + 2];
      min_x = std::min(min_x, vx);
      min_y = std::min(min_y, vy);
      min_z = std::min(min_z, vz);
      max_x = std::max(max_x, vx);
      max_y = std::max(max_y, vy);
      max_z = std::max(max_z, vz);
    }
    uint16_t attribute = 0;
    file.read(reinterpret_cast<char*>(&attribute), sizeof(attribute));
    if (!file)
    {
      return false;
    }
  }
  ex = static_cast<double>(max_x - min_x);
  ey = static_cast<double>(max_y - min_y);
  ez = static_cast<double>(max_z - min_z);
  return ex > 0.0 && ey > 0.0 && ez > 0.0;
}

/*
 * 读取二进制 STL 三角网格到 shape_msgs/Mesh（Humble 无 mesh_resources 字段时使用）。
 */
bool read_stl_binary_shape_mesh(const std::string& path, shape_msgs::msg::Mesh& mesh_out)
{
  mesh_out.vertices.clear();
  mesh_out.triangles.clear();
  std::ifstream file(path, std::ios::binary);
  if (!file)
  {
    return false;
  }
  file.seekg(0, std::ios::end);
  const std::streamoff file_len = file.tellg();
  if (file_len < 84)
  {
    return false;
  }
  file.seekg(0);
  char header[80];
  file.read(header, 80);
  uint32_t tri_count = 0;
  file.read(reinterpret_cast<char*>(&tri_count), sizeof(tri_count));
  const std::streamoff expected = 84 + static_cast<std::streamoff>(tri_count) * 50;
  if (tri_count == 0 || expected != file_len)
  {
    return false;
  }
  for (uint32_t t = 0; t < tri_count; ++t)
  {
    float tri[12];
    file.read(reinterpret_cast<char*>(tri), sizeof(tri));
    if (!file)
    {
      mesh_out.vertices.clear();
      mesh_out.triangles.clear();
      return false;
    }
    const uint32_t base = static_cast<uint32_t>(mesh_out.vertices.size());
    for (int v = 0; v < 3; ++v)
    {
      geometry_msgs::msg::Point p;
      p.x = static_cast<double>(tri[3 + v * 3 + 0]);
      p.y = static_cast<double>(tri[3 + v * 3 + 1]);
      p.z = static_cast<double>(tri[3 + v * 3 + 2]);
      mesh_out.vertices.push_back(p);
    }
    shape_msgs::msg::MeshTriangle mt;
    mt.vertex_indices[0] = base;
    mt.vertex_indices[1] = base + 1;
    mt.vertex_indices[2] = base + 2;
    mesh_out.triangles.push_back(mt);
    uint16_t attribute = 0;
    file.read(reinterpret_cast<char*>(&attribute), sizeof(attribute));
    if (!file)
    {
      mesh_out.vertices.clear();
      mesh_out.triangles.clear();
      return false;
    }
  }
  return !mesh_out.vertices.empty();
}

/*
 * 与 web/js/robot/RobotScene.js 保持一致的 TargetSensor 本体修正（局部欧拉角）：
 * RX=0, RY=pi, RZ=pi。这样前端显示与 MoveIt 碰撞体朝向一致。
 */
geometry_msgs::msg::Pose target_sensor_mesh_local_pose()
{
  const Eigen::Quaterniond q_rx(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()));
  const Eigen::Quaterniond q_ry(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
  const Eigen::Quaterniond q_rz(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond q_mesh = (q_rx * q_ry * q_rz).normalized();
  geometry_msgs::msg::Pose mesh_pose;
  mesh_pose.position.x = 0.0;
  mesh_pose.position.y = 0.0;
  mesh_pose.position.z = 0.0;
  mesh_pose.orientation.x = q_mesh.x();
  mesh_pose.orientation.y = q_mesh.y();
  mesh_pose.orientation.z = q_mesh.z();
  mesh_pose.orientation.w = q_mesh.w();
  return mesh_pose;
}

std::string resolve_target_stl_path()
{
  std::string share;
  try
  {
    share = ament_index_cpp::get_package_share_directory("orion_description");
  }
  catch (const std::exception&)
  {
    return {};
  }
  const std::string primary = share + "/meshes/stl/target.stl";
  std::ifstream check_primary(primary);
  if (check_primary.good())
  {
    return primary;
  }
  const std::string legacy = share + "/target.stl";
  std::ifstream check_legacy(legacy);
  if (check_legacy.good())
  {
    return legacy;
  }
  return {};
}

}  // namespace

/*
 * 将 base 与 local 转为 Eigen Isometry 链乘，再输出合成 Pose（位置+四元数）。
 */
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

/*
 * id=object_id、frame=base_link 的整根缆绳圆柱（与规划默认「单物体」一致）；operation 传 ADD/REMOVE。
 */
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

/*
 * TargetSensor peg：嵌入 STL 三角网格 + mesh_pose（与网页 target.stl 对齐/躺平一致）；无 STL 时退化为定向盒。
 */
moveit_msgs::msg::CollisionObject makeTargetSensorPegCollisionObject(const std::string& object_id,
                                                                       const std::string& frame_id,
                                                                       const geometry_msgs::msg::Pose& object_pose,
                                                                       uint8_t operation)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = object_id;
  object.header.frame_id = frame_id;
  object.pose.position.x = 0.0;
  object.pose.position.y = 0.0;
  object.pose.position.z = 0.0;
  object.pose.orientation.w = 1.0;
  object.pose.orientation.x = 0.0;
  object.pose.orientation.y = 0.0;
  object.pose.orientation.z = 0.0;
  object.operation = operation;

  double ex = 0.15;
  double ey = 0.02;
  double ez = 0.02;
  const std::string stl_path = resolve_target_stl_path();
  shape_msgs::msg::Mesh mesh;
  if (!stl_path.empty())
  {
    if (!read_stl_binary_aabb(stl_path, ex, ey, ez))
    {
      ex = 0.15;
      ey = 0.02;
      ez = 0.02;
    }
    if (!read_stl_binary_shape_mesh(stl_path, mesh))
    {
      mesh.vertices.clear();
      mesh.triangles.clear();
    }
  }
  const geometry_msgs::msg::Pose mesh_local = target_sensor_mesh_local_pose();
  object.mesh_poses.push_back(composePose(object_pose, mesh_local));
  if (!mesh.vertices.empty())
  {
    object.meshes.push_back(std::move(mesh));
  }
  else
  {
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = { static_cast<double>(ex), static_cast<double>(ey), static_cast<double>(ez) };
    object.primitives.push_back(box);
    object.primitive_poses.push_back(composePose(object_pose, mesh_local));
  }
  return object;
}

/*
 * 单段 CableSegment → CollisionObject：圆柱尺寸为 length/radius，轴向由 segment.axis 经 buildCylinderCollisionOrientationFromAxis 对齐局部 Z。
 */
moveit_msgs::msg::CollisionObject makeSegmentCollisionObject(const CableSegment& segment,
                                                             const std::string& frame_id,
                                                             uint8_t operation)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = segment.id;
  obj.header.frame_id = frame_id;

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  prim.dimensions.resize(2);
  prim.dimensions[0] = static_cast<double>(segment.length);
  prim.dimensions[1] = static_cast<double>(segment.radius);
  obj.primitives.push_back(prim);

  geometry_msgs::msg::Pose pose;
  pose.position.x = segment.center.x();
  pose.position.y = segment.center.y();
  pose.position.z = segment.center.z();
  geometry_msgs::msg::Vector3 axis_msg;
  axis_msg.x = segment.axis.x();
  axis_msg.y = segment.axis.y();
  axis_msg.z = segment.axis.z();
  pose.orientation = buildCylinderCollisionOrientationFromAxis(axis_msg);
  obj.primitive_poses.push_back(pose);
  obj.operation = operation;
  return obj;
}

}  // namespace orion_mtc
