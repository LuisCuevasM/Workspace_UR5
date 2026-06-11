from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    grasp_service_name_arg = DeclareLaunchArgument(
        "grasp_service_name",
        default_value="/ur5/prepare_bt_grasp_cycle",
        description="Servicio de bajo nivel que segmenta (SAM2) y genera grasps.",
    )
    pick_service_name_arg = DeclareLaunchArgument(
        "pick_service_name",
        default_value="/ur5/execute_pick",
        description="Servicio de bajo nivel que ejecuta el pick (toma y levanta).",
    )
    place_service_name_arg = DeclareLaunchArgument(
        "place_service_name",
        default_value="/ur5/execute_place",
        description="Servicio de bajo nivel que ejecuta el place (deposito y home).",
    )

    skill_server_node = Node(
        package="ur5_description",
        executable="ur5_skill_server.py",
        name="ur5_skill_server",
        output="screen",
        parameters=[
            {
                "grasp_service_name": LaunchConfiguration("grasp_service_name"),
                "pick_service_name": LaunchConfiguration("pick_service_name"),
                "place_service_name": LaunchConfiguration("place_service_name"),
            }
        ],
    )

    return LaunchDescription(
        [
            grasp_service_name_arg,
            pick_service_name_arg,
            place_service_name_arg,
            skill_server_node,
        ]
    )
