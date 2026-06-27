cmake_minimum_required(VERSION 3.8)
project(ur5_manipulation)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetVlmInstruction.srv"
  "srv/GetCameraImage.srv"
  "srv/Perceive.srv"
  "srv/QueryInstruction.srv"
  "srv/DecodeSkill.srv"
  "srv/ExecuteInstruction.srv"
  "srv/RunSkill.srv"
)

install(PROGRAMS
  scripts/VLM_prompt.py
  scripts/wait_for_services.py
  scripts/test_pick_place.py
  src/detection_grasp.py
  src/grasp_sensor.py
  src/robot_perception_node.py
  src/robot_skill_node.py
  src/skill_decoder_node.py
  src/vlm_instruction_query_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch scripts srv
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
