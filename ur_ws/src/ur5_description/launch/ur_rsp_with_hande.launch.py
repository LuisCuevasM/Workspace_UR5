from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, FindExecutable

def generate_launch_description():

    robot_ip = LaunchConfiguration("robot_ip")
    ur_type = LaunchConfiguration("ur_type")
    tf_prefix = LaunchConfiguration("tf_prefix")

    description_file = PathJoinSubstitution([
        FindPackageShare("ur5_description"),
        "urdf",
        "ur5_with_hande.urdf.xacro"
    ])

    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        description_file, " ",
        "robot_ip:=", robot_ip, " ",
        "ur_type:=", ur_type, " ",
        "tf_prefix:=", tf_prefix, " ",
        "use_hande:=true"
    ])

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        )
    ])
