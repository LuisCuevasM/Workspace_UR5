from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gripper_config_arg = DeclareLaunchArgument(
        "gripper_config",
        default_value="",
        description="Ruta al archivo de configuracion del gripper para GraspGen.",
    )

    grasp_threshold_arg = DeclareLaunchArgument(
        "grasp_threshold",
        default_value="0.6",
        description="Umbral de score para GraspGen.",
    )

    num_grasps_arg = DeclareLaunchArgument(
        "num_grasps",
        default_value="200",
        description="Cantidad de grasps a muestrear en GraspGen.",
    )

    max_retries_arg = DeclareLaunchArgument(
        "max_retries",
        default_value="3",
        description="Cantidad maxima de reintentos del Behavior Tree.",
    )

    sam_node = ExecuteProcess(
        cmd=["python3", "/workspace/src/sam_node/sam2_click_node.py"],
        output="screen",
    )

    graspgen_node = ExecuteProcess(
        cmd=[
            "python3",
            "/workspace/src/graspgen_rviz/graspgen_rviz/graspgen_rviz_infer_node.py",
            "--ros-args",
            "-p",
            "object_topic:=/sam2/object_cloud",
            "-p",
            "scene_topic:=/sam2/scene_cloud_no_object",
            "-p",
            "infer_service_name:=/graspgen/run_inference",
            "-p",
            "auto_infer:=false",
            "-p",
            ["gripper_config:=", LaunchConfiguration("gripper_config")],
            "-p",
            ["grasp_threshold:=", LaunchConfiguration("grasp_threshold")],
            "-p",
            ["num_grasps:=", LaunchConfiguration("num_grasps")],
            "-p",
            "top_grasp_count:=5",
            "-p",
            "publish_all_grasps:=false",
        ],
        output="screen",
    )

    move_group_client_node = Node(
        package="ur5_description",
        executable="ur5_move_group_client.py",
        name="minimal_grasp_executor",
        output="screen",
        parameters=[
            {
                "auto_start": False,
                "execute": True,
                "pause_sam_during_pick": True,
                "grasp_service_name": "/graspgen/run_inference",
                "sam_select_service_name": "/sam2/select_object",
                "sam_pause_service_name": "/sam2/pause_inference",
                "sam_resume_service_name": "/sam2/resume_inference",
                "run_cycle_service_name": "/ur5/run_grasp_cycle",
                "execute_cached_cycle_service_name": "/ur5/execute_cached_grasp_cycle",
                "go_home_service_name": "/ur5/go_home",
                "top_grasps_topic": "/graspgen/top_grasps",
            }
        ],
    )

    octomap_home_cloud_manager_node = Node(
        package="ur5_description",
        executable="octomap_home_cloud_manager.py",
        name="octomap_home_cloud_manager",
        output="screen",
        parameters=[
            {
                "input_cloud_topic": "/sam2/scene_cloud_no_object",
                "output_cloud_topic": "/octomap/home_scene_cloud",
                "clear_service_name": "/clear_octomap",
                "publish_rate_hz": 2.0,
            }
        ],
    )

    bt_node = Node(
        package="ur5_description",
        executable="ur5_behavior_tree_node.py",
        name="ur5_bt_grasp_coordinator",
        output="screen",
        parameters=[
            {
                "sam_select_service_name": "/sam2/select_object",
                "sam_pause_service_name": "/sam2/pause_inference",
                "grasp_service_name": "/graspgen/run_inference",
                "execute_cached_cycle_service_name": "/ur5/execute_cached_grasp_cycle",
                "go_home_service_name": "/ur5/go_home",
                "prepare_bt_service_name": "/ur5/prepare_bt_grasp_cycle",
                "execute_bt_service_name": "/ur5/execute_bt_grasp_cycle",
                "bt_service_name": "/ur5/run_bt_grasp_cycle",
                "top_grasps_topic": "/graspgen/top_grasps",
                "max_retries": LaunchConfiguration("max_retries"),
            }
        ],
    )

    return LaunchDescription(
        [
            gripper_config_arg,
            grasp_threshold_arg,
            num_grasps_arg,
            max_retries_arg,
            sam_node,
            graspgen_node,
            move_group_client_node,
            octomap_home_cloud_manager_node,
            bt_node,
        ]
    )
