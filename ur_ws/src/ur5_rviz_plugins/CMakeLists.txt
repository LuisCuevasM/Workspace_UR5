cmake_minimum_required(VERSION 3.8)
project(ur5_rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ur5_manipulation REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

set(headers_to_moc
  include/ur5_rviz_plugins/grasp_control_panel.hpp
  include/ur5_rviz_plugins/object_detected_panel.hpp
  include/ur5_rviz_plugins/skill_control_panel.hpp
)

qt5_wrap_cpp(moc_files ${headers_to_moc})

add_library(${PROJECT_NAME} SHARED
  ${moc_files}
  src/grasp_control_panel.cpp
  src/object_detected_panel.cpp
  src/skill_control_panel.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  pluginlib
  rclcpp
  rviz_common
  std_msgs
  std_srvs
  ur5_manipulation
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(pluginlib rclcpp rviz_common std_msgs std_srvs ur5_manipulation)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
