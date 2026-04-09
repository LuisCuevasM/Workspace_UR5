cmake_minimum_required(VERSION 3.8)
project(ur5_description)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(xacro REQUIRED)

add_executable(grasp2moveit src/grasp2moveit.cpp)
ament_target_dependencies(
  grasp2moveit
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  std_srvs
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  grasp2moveit
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/ur5_move_group_client.py
  src/ur5_square_points_pub.py
  src/gripper_jointstate.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf meshes models config rviz worlds launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
