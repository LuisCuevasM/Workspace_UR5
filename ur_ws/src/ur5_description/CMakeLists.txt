cmake_minimum_required(VERSION 3.8)
project(ur5_description)

# 1. Encontrar paquetes necesarios
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(xacro REQUIRED)

# --- SECCIÓN DE C++ REMOVIDA TEMPORALMENTE ---
# Aquí es donde fallaba. No puedes poner ament_target_dependencies 
# ni install(TARGETS) si no has definido un add_executable antes.

# 2. Instalar scripts de Python
# Estos no necesitan compilación, solo se copian a la carpeta de instalación
install(PROGRAMS
  src/ur5_behavior_tree_node.py
  src/ur5_move_group_client.py
  src/ur5_place.py
  src/ur5_return_home_on_success.py
  src/gripper_jointstate.py
  src/ntfields_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3. Instalar directorios de recursos
install(
  DIRECTORY urdf models config rviz worlds launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
