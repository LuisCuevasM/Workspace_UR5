from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(pkg_name, relative_path):
    pkg_path = get_package_share_directory(pkg_name)
    abs_path = os.path.join(pkg_path, relative_path)
    with open(abs_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")

    planning_group = LaunchConfiguration("planning_group")
    base_frame = LaunchConfiguration("base_frame")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    execute = LaunchConfiguration("execute")

    # -------- URDF desde xacro (ur_description) --------
    ur_description_share = get_package_share_directory("ur_description")

    # En UR ROS2 normalmente existe: urdf/ur.urdf.xacro
    urdf_xacro = os.path.join(ur_description_share, "urdf", "ur5.urdf.xacro")

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", urdf_xacro]),
            value_type=str,
        )
    }

    # -------- SRDF desde xacro (ur_moveit_config) --------
    ur_moveit_share = get_package_share_directory("ur_moveit_config")
    srdf_xacro = os.path.join(ur_moveit_share, "srdf", "ur.srdf.xacro")

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(["xacro ", srdf_xacro, " ur_type:=ur5"]),
            value_type=str,
        )
    }

    # -------- YAMLs de MoveIt --------
    ompl_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    kinematics_yaml = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("ur_moveit_config", "config/joint_limits.yaml")

    planning_pipelines = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_yaml,
    }

    robot_description_planning = {"robot_description_planning": joint_limits_yaml}
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    node = Node(
        package="ur5_description",
        executable="ur5_pose_goal_node.py",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipelines,
            {
                "planning_group": planning_group,
                "base_frame": base_frame,
                "x": x, "y": y, "z": z,
                "roll": roll, "pitch": pitch, "yaw": yaw,
                "execute": execute,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur5"),
        DeclareLaunchArgument("planning_group", default_value="ur_manipulator"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),

        DeclareLaunchArgument("x", default_value="0.45"),
        DeclareLaunchArgument("y", default_value="0.10"),
        DeclareLaunchArgument("z", default_value="0.25"),
        DeclareLaunchArgument("roll", default_value="0.0"),
        DeclareLaunchArgument("pitch", default_value="1.57"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("execute", default_value="true"),

        node
    ])

