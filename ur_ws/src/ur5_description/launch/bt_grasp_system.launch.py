from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sam_select_service_name_arg = DeclareLaunchArgument(
        "sam_select_service_name",
        default_value="/sam2/select_object",
        description="Servicio usado para seleccionar el objeto en SAM2.",
    )

    vlm_health_url_arg = DeclareLaunchArgument(
        "vlm_health_url",
        default_value="http://192.168.1.110:8888/health",
        description="URL del endpoint /health del servidor VLM.",
    )

    vlm_release_url_arg = DeclareLaunchArgument(
        "vlm_release_url",
        default_value="http://192.168.1.110:8888/release",
        description="URL del endpoint /release para liberar la GPU del modelo VLM.",
    )

    vlm_select_service_name_arg = DeclareLaunchArgument(
        "vlm_select_service_name",
        default_value="/vlm_prompt/select_object",
        description="Servicio ROS2 del nodo VLM para seleccion semantica.",
    )

    vlm_health_timeout_sec_arg = DeclareLaunchArgument(
        "vlm_health_timeout_sec",
        default_value="60.0",
        description="Segundos maximos esperando que el servidor VLM responda /health.",
    )

    vlm_release_timeout_sec_arg = DeclareLaunchArgument(
        "vlm_release_timeout_sec",
        default_value="20.0",
        description="Segundos maximos esperando la respuesta de /release del servidor VLM.",
    )

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

    sam_selection_retries_arg = DeclareLaunchArgument(
        "sam_selection_retries",
        default_value="3",
        description="Cantidad maxima de reintentos si falla la seleccion de objeto en SAM2.",
    )

    clean_table_max_cycles_arg = DeclareLaunchArgument(
        "clean_table_max_cycles",
        default_value="20",
        description="Cantidad maxima de objetos/ciclos para el BT Limpiar la Mesa.",
    )

    clean_table_use_vlm_arg = DeclareLaunchArgument(
        "clean_table_use_vlm",
        default_value="true",
        description="Usar seleccion VLM en el BT Limpiar la Mesa.",
    )

    clean_table_scan_on_no_grasps_arg = DeclareLaunchArgument(
        "clean_table_scan_on_no_grasps",
        default_value="true",
        description="Usar scanner como recuperacion si no se generan grasps.",
    )

    max_grasps_to_try_arg = DeclareLaunchArgument(
        "max_grasps_to_try",
        default_value="10",
        description="Cantidad maxima de candidatos publicados y evaluados por ciclo.",
    )

    max_graspgen_retries_arg = DeclareLaunchArgument(
        "max_graspgen_retries",
        default_value="3",
        description="Cantidad maxima de regeneraciones GraspGen tras fallo IK/approach.",
    )

    collision_threshold_arg = DeclareLaunchArgument(
        "collision_threshold",
        default_value="0.002",
        description="Distancia minima gripper-escena para considerar colision, en metros.",
    )

    collision_filter_retries_arg = DeclareLaunchArgument(
        "collision_filter_retries",
        default_value="2",
        description="Reintentos internos relajando el umbral si todos los grasps colisionan.",
    )

    gripper_control_node = Node(
        package="ur5_description",
        executable="ur5_gripper_control_node.py",
        name="ur5_gripper_control",
        output="screen",
        parameters=[
            {
                "gripper_action_name": "gripper/gripper_action_controller/gripper_cmd",
                "close_gripper_position": 0.0,
                "close_gripper_max_effort": 0.0,
                "close_gripper_velocity": 0.05,
                "close_gripper_force": 1.5,
                "open_gripper_position": 0.025,
                "open_gripper_max_effort": 0.0,
                "open_gripper_velocity": 0.05,
                "open_gripper_force": 1.0,
                "velocity_command_topic": "/gripper/gripper_velocity_controller/commands",
                "force_command_topic": "/gripper/gripper_force_controller/commands",
                "config_topic": "/ur5/gripper/config",
                "open_service_name": "/ur5/gripper/open",
                "close_service_name": "/ur5/gripper/close",
            }
        ],
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
            "scanned_object_topic:=/point_cloud_scanner/object_cloud_complete",
            "-p",
            "scene_topic:=/sam2/scene_cloud_no_object",
            "-p",
            "infer_service_name:=/graspgen/run_inference",
            "-p",
            "infer_sam_service_name:=/graspgen/run_inference_sam",
            "-p",
            "auto_infer:=false",
            "-p",
            ["gripper_config:=", LaunchConfiguration("gripper_config")],
            "-p",
            ["grasp_threshold:=", LaunchConfiguration("grasp_threshold")],
            "-p",
            ["num_grasps:=", LaunchConfiguration("num_grasps")],
            "-p",
            ["collision_threshold:=", LaunchConfiguration("collision_threshold")],
            "-p",
            ["collision_filter_retries:=", LaunchConfiguration("collision_filter_retries")],
            "-p",
            ["top_grasp_count:=", LaunchConfiguration("max_grasps_to_try")],
            "-p",
            "publish_all_grasps:=false",
        ],
        output="screen",
        respawn=True,
        respawn_delay=5.0,
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
                "sam_select_service_name": LaunchConfiguration("sam_select_service_name"),
                "sam_pause_service_name": "/sam2/pause_inference",
                "sam_resume_service_name": "/sam2/resume_inference",
                "run_cycle_service_name": "/ur5/run_grasp_cycle",
                "execute_cached_cycle_service_name": "/ur5/execute_cached_grasp_cycle",
                "clear_grasp_cache_service_name": "/ur5/clear_grasp_cache",
                "go_home_service_name": "/ur5/go_home",
                "open_gripper_service_name": "/ur5/gripper/open",
                "close_gripper_service_name": "/ur5/gripper/close",
                "top_grasps_topic": "/graspgen/top_grasps",
                "top_scores_topic": "/graspgen/top_scores",
                "max_grasps_to_try": LaunchConfiguration("max_grasps_to_try"),
                "max_graspgen_retries": LaunchConfiguration("max_graspgen_retries"),
                "use_cached_grasps_first": True,
                "clear_cache_on_ik_failure": True,
                "cached_grasp_max_age_sec": 120.0,
                "planning_frame": "base_link",
                "approach_distance": 0.08,
                "ik_timeout": 0.25,
                "planning_timeout": 25.0,
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
                "sam_select_service_name": LaunchConfiguration("sam_select_service_name"),
                "sam_pause_service_name": "/sam2/pause_inference",
                "grasp_service_name": "/graspgen/run_inference",
                "grasp_sam_service_name": "/graspgen/run_inference_sam",
                "execute_cached_cycle_service_name": "/ur5/execute_cached_grasp_cycle",
                "clear_grasp_cache_service_name": "/ur5/clear_grasp_cache",
                "go_home_service_name": "/ur5/go_home",
                "prepare_bt_service_name": "/ur5/prepare_bt_grasp_cycle",
                "prepare_context_bt_service_name": "/ur5/prepare_context_bt_grasp_cycle",
                "execute_bt_service_name": "/ur5/execute_bt_grasp_cycle",
                "bt_service_name": "/ur5/run_bt_grasp_cycle",
                "clean_table_service_name": "/ur5/clean_table",
                "execute_pick_service_name": "/ur5/execute_pick",
                "execute_place_service_name": "/ur5/execute_place",
                "clean_table_max_cycles": LaunchConfiguration("clean_table_max_cycles"),
                "clean_table_use_vlm": LaunchConfiguration("clean_table_use_vlm"),
                "clean_table_scan_on_no_grasps": LaunchConfiguration("clean_table_scan_on_no_grasps"),
                "scan_prepare_bt_service_name": "/ur5/prepare_scan_bt_grasp_cycle",
                "top_grasps_topic": "/graspgen/top_grasps",
                "max_retries": LaunchConfiguration("max_retries"),
                "sam_selection_retries": LaunchConfiguration("sam_selection_retries"),
                "service_wait_timeout_sec": 10.0,
                "vlm_health_url": LaunchConfiguration("vlm_health_url"),
                "vlm_release_url": LaunchConfiguration("vlm_release_url"),
                "vlm_select_service_name": LaunchConfiguration("vlm_select_service_name"),
                "vlm_health_timeout_sec": LaunchConfiguration("vlm_health_timeout_sec"),
                "vlm_release_timeout_sec": LaunchConfiguration("vlm_release_timeout_sec"),
                "prepare_vlm_bt_service_name": "/ur5/prepare_vlm_bt_grasp_cycle",
                "scan_prepare_vlm_bt_service_name": "/ur5/prepare_scan_vlm_bt_grasp_cycle",
            }
        ],
    )

    return LaunchDescription(
        [
            sam_select_service_name_arg,
            vlm_health_url_arg,
            vlm_release_url_arg,
            vlm_select_service_name_arg,
            vlm_health_timeout_sec_arg,
            vlm_release_timeout_sec_arg,
            gripper_config_arg,
            grasp_threshold_arg,
            num_grasps_arg,
            max_retries_arg,
            sam_selection_retries_arg,
            clean_table_max_cycles_arg,
            clean_table_use_vlm_arg,
            clean_table_scan_on_no_grasps_arg,
            max_grasps_to_try_arg,
            max_graspgen_retries_arg,
            collision_threshold_arg,
            collision_filter_retries_arg,
            gripper_control_node,
            sam_node,
            graspgen_node,
            move_group_client_node,
            octomap_home_cloud_manager_node,
            bt_node,
        ]
    )
