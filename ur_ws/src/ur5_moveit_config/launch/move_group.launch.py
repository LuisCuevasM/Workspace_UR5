import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("ur5_full", package_name="ur5_moveit_config")
        .robot_description(file_path="config/ur5_full.urdf.xacro")
        .robot_description_semantic(file_path="config/ur5_full.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )
    
    ompl_pipeline_params = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": " ".join([
                "default_planner_request_adapters/AddTimeOptimalParameterization",
                "default_planner_request_adapters/FixWorkspaceBounds",
                "default_planner_request_adapters/FixStartStateBounds",
                "default_planner_request_adapters/FixStartStateCollision",
                "default_planner_request_adapters/FixStartStatePathConstraints",
            ]),
            "start_state_max_bounds_error": 0.1,
        }
    }

    # ── 1. move_group ──────────────────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"default_planning_pipeline": "ompl"},
            ompl_pipeline_params,
        ],
    )

    # ── 2. robot_state_publisher ───────────────────────────────────────────
    # Publica /robot_description y los TF del robot; RViz lo necesita
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],  # solo el URDF
    )

    # ── 3. RViz con la config de MoveIt ────────────────────────────────────
    rviz_config_file = os.path.join(
        get_package_share_directory("ur5_moveit_config"),
        "config",
        "moveit.rviz",   # ← tu archivo .rviz; ajusta el nombre si es distinto
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,           # /robot_description
            moveit_config.robot_description_semantic,  # /robot_description_semantic
            moveit_config.robot_description_kinematics,# cinemática para el plugin
            moveit_config.planning_pipelines,          # pipelines disponibles
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
    ])