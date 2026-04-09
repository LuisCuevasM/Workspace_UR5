from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    tf_prefix = LaunchConfiguration("tf_prefix")
    robot_ip = LaunchConfiguration("robot_ip")
    controllers_yaml = LaunchConfiguration("controllers_yaml")

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur5_description"), "urdf", "ur5_with_hande.urdf.xacro"]),
            " ",
            "ur_type:=", ur_type, " ",
            "tf_prefix:=", tf_prefix, " ",
            "robot_ip:=", robot_ip, " ",
            "use_mock_hardware:=false ",
            "mock_sensor_commands:=false ",
            "headless_mode:=false ",
        ]),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    # ros2_control controller manager (UR driver real) + controllers from yaml
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            controllers_yaml,
        ],
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    ur_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Robot state publisher (TF)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur5"),
        DeclareLaunchArgument("tf_prefix", default_value="ur5_"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.56.101"),
        DeclareLaunchArgument(
            "controllers_yaml",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur5_description"), "config", "controllers_mock.yaml"]
            ),
        ),

        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        ur_arm_controller_spawner,
        rviz_node,
    ])
