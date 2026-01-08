from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    robot_ip = LaunchConfiguration("robot_ip")
    ur_type = LaunchConfiguration("ur_type")

    description_pkg = FindPackageShare("ur5_description")
    xacro_file = os.path.join(
        description_pkg.find("ur5_description"),
        "urdf",
        "ur5_with_hande.urdf.xacro"
    )

    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            xacro_file,
            " robot_ip:=", robot_ip,
            " ur_type:=", ur_type,
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.56.101",
            description="IP address of the UR robot"
        ),
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5",
            description="UR robot type"
        ),
        robot_state_publisher_node,
    ])
