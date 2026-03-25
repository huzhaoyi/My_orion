cmake_minimum_required(VERSION 3.8)
project(orion_mtc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(orion_mtc_msgs REQUIRED)
find_package(sealien_ctrlpilot_msgmanagement REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)

set(ORION_MTC_SOURCES
  src/main.cpp
  src/app/orion_mtc_node.cpp
  src/interface/manipulator_ros_interface.cpp
  src/core/constants.cpp
  src/config/mtc_config.cpp
  src/config/runtime_policy_loader.cpp
  src/decision/feasibility_checker.cpp
  src/decision/reach_kinematics.cpp
  src/decision/cylinder_side_grasp.cpp
  src/decision/cable_side_pick_precheck.cpp
  src/decision/target_selector.cpp
  src/planning/collision_object_utils.cpp
  src/planning/cable_segments.cpp
  src/planning/cable_side_grasp.cpp
  src/planning/pick_task_builder.cpp
  src/scene/planning_scene_manager.cpp
  src/execution/trajectory_executor.cpp
  src/execution/solution_executor.cpp
  src/perception/pose_cache.cpp
  src/perception/vector3_cache.cpp
  src/perception/target_cache.cpp
  src/perception/perception_snapshot_provider.cpp
  src/orchestration/task_queue.cpp
  src/orchestration/recovery_actions.cpp
  src/orchestration/job_deduplicator.cpp
  src/orchestration/task_manager.cpp
)

add_executable(keypoint_to_arm_tf_node src/keypoint_to_arm_tf_node.cpp)
ament_target_dependencies(keypoint_to_arm_tf_node
  rclcpp
  geometry_msgs
  sealien_ctrlpilot_msgmanagement
  tf2_ros
  tf2_geometry_msgs
  tf2
)

add_executable(mtc_node ${ORION_MTC_SOURCES})
target_include_directories(mtc_node PRIVATE
  include
  ${Eigen3_INCLUDE_DIRS}
)
ament_target_dependencies(mtc_node
  builtin_interfaces
  rclcpp
  moveit_task_constructor_core
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning
  sensor_msgs
  geometry_msgs
  shape_msgs
  moveit_msgs
  moveit_task_constructor_msgs
  control_msgs
  rclcpp_action
  trajectory_msgs
  std_msgs
  std_srvs
  orion_mtc_msgs
  tf2_ros
)

install(TARGETS mtc_node keypoint_to_arm_tf_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
