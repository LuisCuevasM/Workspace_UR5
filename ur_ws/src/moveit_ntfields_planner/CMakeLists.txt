cmake_minimum_required(VERSION 3.8)
project(moveit_ntfields_planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(CURL REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ntfields_planner_manager.cpp
  src/ntfields_planning_context.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CURL_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  moveit_core
  moveit_msgs
  moveit_ros_planning
  pluginlib
  rclcpp
)

target_link_libraries(${PROJECT_NAME}
  ${CURL_LIBRARIES}
  JsonCpp::JsonCpp
)

pluginlib_export_plugin_description_file(moveit_core plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  CURL
  jsoncpp
  moveit_core
  moveit_msgs
  moveit_ros_planning
  pluginlib
  rclcpp
)

ament_package()
