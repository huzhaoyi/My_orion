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
find_package(orion_mtc_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(Eigen3 REQUIRED)

set(ORION_MTC_SOURCES
  src/main.cpp
  src/app/orion_mtc_node.cpp
  src/core/constants.cpp
  src/config/mtc_config.cpp
  src/config/runtime_policy_loader.cpp
  src/planning/collision_object_utils.cpp
  src/planning/pick_task_builder.cpp
  src/planning/place_task_builder.cpp
  src/planning/place_release_task_builder.cpp
  src/planning/place_generator.cpp
  src/scene/planning_scene_manager.cpp
  src/execution/trajectory_executor.cpp
  src/execution/solution_executor.cpp
  src/perception/pose_cache.cpp
  src/perception/target_cache.cpp
  src/perception/target_selector.cpp
  src/perception/grasp_generator.cpp
  src/orchestration/task_queue.cpp
  src/orchestration/recovery_actions.cpp
  src/orchestration/task_manager.cpp
)

add_executable(mtc_node ${ORION_MTC_SOURCES})
target_include_directories(mtc_node PRIVATE
  include
  ${Eigen3_INCLUDE_DIRS}
)
ament_target_dependencies(mtc_node
  rclcpp
  moveit_task_constructor_core
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  shape_msgs
  moveit_msgs
  moveit_task_constructor_msgs
  control_msgs
  rclcpp_action
  trajectory_msgs
  std_msgs
  orion_mtc_msgs
)

install(TARGETS mtc_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
