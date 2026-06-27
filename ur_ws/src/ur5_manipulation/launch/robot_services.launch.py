"""Levanta la capa de servicios del contrato del orquestador LangGraph.

    robot_perception_node  -> /robot/get_camera_image, /robot/perceive
    robot_skill_node       -> /robot/clean_table|grasp|pick|place|pick_place|push|handover
    skill_decoder_node     -> /ur5/decode_skill
                              /ur5/execute_instruction

Asume que el stack de bajo nivel (driver UR5 + MoveIt + BT + SAM2 + GraspGen +
RealSense) ya esta corriendo, p.ej. via ur5_manipulation.launch.py. Estos nodos
solo orquestan los servicios Trigger existentes y exponen el contrato tipado.

Ejemplo:
    ros2 launch ur5_manipulation robot_services.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base_frame = LaunchConfiguration("base_frame")

    perception_node = Node(
        package="ur5_manipulation",
        executable="robot_perception_node.py",
        name="robot_perception_node",
        output="screen",
        parameters=[{"base_frame": base_frame}],
    )

    skill_node = Node(
        package="ur5_manipulation",
        executable="robot_skill_node.py",
        name="robot_skill_node",
        output="screen",
    )

    skill_decoder_node = Node(
        package="ur5_manipulation",
        executable="skill_decoder_node.py",
        name="skill_decoder_node",
        output="screen",
        parameters=[
            {
                "service_name": "/ur5/decode_skill",
                "execute_instruction_service_name": "/ur5/execute_instruction",
                "query_instruction_service_name": "/ur5/query_instruction",
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            perception_node,
            skill_node,
            skill_decoder_node,
        ]
    )
