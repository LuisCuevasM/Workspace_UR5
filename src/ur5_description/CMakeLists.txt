cmake_minimum_required(VERSION 3.8)
project(ur5_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY urdf meshes models config rviz worlds launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

