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

add_executable(mtc_node src/mtc_node.cpp)
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
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/test_pick_then_place.sh
  DESTINATION share/${PROJECT_NAME}/scripts)

ament_package()
