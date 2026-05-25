cmake_minimum_required(VERSION 3.8)
project(sealien_ctrlpilot_manipulator_orion)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
