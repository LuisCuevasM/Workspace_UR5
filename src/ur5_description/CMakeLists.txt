cmake_minimum_required(VERSION 3.8)
project(ur5_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)


install(PROGRAMS
  src/ur5_move_group_client.py
  src/ur5_square_points_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf meshes models config rviz worlds launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

