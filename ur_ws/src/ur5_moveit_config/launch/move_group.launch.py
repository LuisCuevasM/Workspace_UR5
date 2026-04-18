import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory("ur5_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder("ur5_full", package_name="ur5_moveit_config")
        .robot_description(file_path="config/ur5_full.urdf.xacro")
        .robot_description_semantic(file_path="config/ur5_full.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "ntfields"],
            default_planning_pipeline="ntfields",
        )
        .to_moveit_configs()
    )

    # ── OMPL como planificador base + CHOMP como post-procesador ──────────
    # El orden de request_adapters importa:
    #   1. OMPL planifica la trayectoria inicial
    #   2. chomp/OptimizerAdapter toma esa trayectoria y la optimiza
    #   3. AddTimeOptimalParameterization añade perfiles de velocidad/aceleración
    #
    # IMPORTANTE: en MoveIt 2 / ROS 2 Humble el adaptador de CHOMP
    # se llama "chomp/OptimizerAdapter" (igual que ROS 1 en este caso),
    # pero debe ir ANTES de AddTimeOptimalParameterization para que
    # la parametrización temporal se aplique sobre la trayectoria ya optimizada.
    ompl_pipeline_params = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": " ".join([
                # 1° CHOMP optimiza la trayectoria cruda de OMPL
                "chomp/OptimizerAdapter",
                # 2° luego se añade la parametrización temporal
                "default_planner_request_adapters/AddTimeOptimalParameterization",
                "default_planner_request_adapters/FixWorkspaceBounds",
                "default_planner_request_adapters/FixStartStateBounds",
                "default_planner_request_adapters/FixStartStateCollision",
                "default_planner_request_adapters/FixStartStatePathConstraints",
            ]),
            "start_state_max_bounds_error": 0.1,
        }
    }

    ntfields_pipeline_params = {
        "ntfields": {
            "planning_plugin": "moveit_ntfields_planner/NTFieldsPlanner",
            "planner_url": "http://172.19.0.2:8888/plan",
            "request_timeout": 60.0,
            "default_dt": 0.15,
            "interpolation_steps": 10,
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

    # ── Cargar parámetros de CHOMP desde YAML ─────────────────────────────
    # Necesario para que el OptimizerAdapter encuentre su configuración
    chomp_yaml_path = os.path.join(pkg, "config", "chomp_planning.yaml")
    with open(chomp_yaml_path, "r") as f:
        chomp_config = yaml.safe_load(f)

    # ── Octomap ───────────────────────────────────────────────────────────
    sensors_yaml_path = os.path.join(pkg, "config", "sensors_3d.yaml")
    with open(sensors_yaml_path, "r") as f:
        octomap_config = yaml.safe_load(f)

    # ── 1. move_group ─────────────────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"default_planning_pipeline": "ntfields"},
            ompl_pipeline_params,
            ntfields_pipeline_params,
            chomp_config,       # ← parámetros que lee el OptimizerAdapter
            octomap_config,
        ],
    )

    # ── 2. robot_state_publisher ──────────────────────────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # ── 3. RViz ───────────────────────────────────────────────────────────
    rviz_config_file = os.path.join(pkg, "config", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
    ])