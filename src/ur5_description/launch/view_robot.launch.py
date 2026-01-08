from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ---------------- robot_description (xacro -> urdf) ----------------
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("ur5_description"), "urdf", "ur5_with_hande.urdf.xacro"]
                ),
                " ",
                "name:=",
                "ur"
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # ---------------- RViz config ----------------
    # If you don't have a config file yet, we can just start RViz without -d.
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur5_description"), "rviz", "view_robot.rviz"]
    )

    # ---------------- Nodes ----------------
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # If you don't have rviz/view_robot.rviz, comment this arguments line.
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
