from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [
            FindPackageShare("point_cloud_scanner"),
            "config",
            "scanner_params.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="point_cloud_scanner",
                executable="point_cloud_scanner_node.py",
                name="point_cloud_scanner",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="point_cloud_scanner",
                executable="scan_motion_executor_node.py",
                name="scan_motion_executor",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
