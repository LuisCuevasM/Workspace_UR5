from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    name = LaunchConfiguration("name")
    ur_type = LaunchConfiguration("ur_type")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_hande = LaunchConfiguration("use_hande")
    hande_prefix = LaunchConfiguration("hande_prefix")
    robot_ns = LaunchConfiguration("robot_ns")

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("ur5_description"),
                "urdf",
                "ur5_with_hande_visual.urdf.xacro",
            ]),
            " ",
            "name:=", name, " ",
            "ur_type:=", ur_type, " ",
            "tf_prefix:=", tf_prefix, " ",
            "use_hande:=", use_hande, " ",
            "hande_prefix:=", hande_prefix, " ",
        ]),
        value_type=str
    )

    robot_description = {"robot_description": robot_description_content}

    return LaunchDescription([
        DeclareLaunchArgument("name", default_value="ur5"),
        DeclareLaunchArgument("ur_type", default_value="ur5"),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("use_hande", default_value="true"),
        DeclareLaunchArgument("hande_prefix", default_value="hande_"),
        DeclareLaunchArgument("robot_ns", default_value="ur5"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot_ns,
            output="both",
            parameters=[robot_description],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
        ),
    ])
