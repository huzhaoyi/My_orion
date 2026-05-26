cmake_minimum_required(VERSION 3.8)
project(sealien_ctrlpilot_manipulator_orion_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  meshes
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/target_new.stl")
  install(FILES target_new.stl DESTINATION share/${PROJECT_NAME}/meshes/stl)
endif()

ament_package()
