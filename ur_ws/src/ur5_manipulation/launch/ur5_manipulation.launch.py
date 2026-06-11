from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    tty_port = LaunchConfiguration("tty_port")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    gripper_config = LaunchConfiguration("gripper_config")
    grasp_threshold = LaunchConfiguration("grasp_threshold")
    num_grasps = LaunchConfiguration("num_grasps")
    max_retries = LaunchConfiguration("max_retries")
    max_grasps_to_try = LaunchConfiguration("max_grasps_to_try")
    max_graspgen_retries = LaunchConfiguration("max_graspgen_retries")
    collision_threshold = LaunchConfiguration("collision_threshold")
    collision_filter_retries = LaunchConfiguration("collision_filter_retries")
    readiness_timeout = LaunchConfiguration("readiness_timeout")
    driver_ready_services = LaunchConfiguration("driver_ready_services")
    bt_ready_services = LaunchConfiguration("bt_ready_services")
    bt_settle_delay = LaunchConfiguration("bt_settle_delay")
    enable_vlm_prompt = LaunchConfiguration("enable_vlm_prompt")
    vlm_instruction = LaunchConfiguration("vlm_instruction")
    vlm_server_url = LaunchConfiguration("vlm_server_url")
    vlm_request_timeout_sec = LaunchConfiguration("vlm_request_timeout_sec")
    sam_apply_timeout_sec = LaunchConfiguration("sam_apply_timeout_sec")
    sam_select_service_name = LaunchConfiguration("sam_select_service_name")

    ur5_manipulation_share = FindPackageShare("ur5_manipulation")
    ur5_description_share = FindPackageShare("ur5_description")
    point_cloud_scanner_share = FindPackageShare("point_cloud_scanner")

    wait_for_services = PathJoinSubstitution(
        [ur5_manipulation_share, "scripts", "wait_for_services.py"]
    )
    driver_launch = PathJoinSubstitution(
        [ur5_description_share, "launch", "ur_driver_with_hande.launch.py"]
    )
    bt_launch = PathJoinSubstitution(
        [ur5_description_share, "launch", "bt_grasp_system.launch.py"]
    )
    scanner_launch = PathJoinSubstitution(
        [point_cloud_scanner_share, "launch", "point_cloud_scanner.launch.py"]
    )

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(driver_launch),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "launch_rviz": launch_rviz,
            "use_fake_hardware": use_fake_hardware,
            "tty_port": tty_port,
            "warehouse_sqlite_path": warehouse_sqlite_path,
            "use_sim_time": use_sim_time,
            "kinematics_params_file": kinematics_params_file,
        }.items(),
    )

    bt_grasp_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bt_launch),
        launch_arguments={
            "gripper_config": gripper_config,
            "grasp_threshold": grasp_threshold,
            "num_grasps": num_grasps,
            "max_retries": max_retries,
            "max_grasps_to_try": max_grasps_to_try,
            "max_graspgen_retries": max_graspgen_retries,
            "collision_threshold": collision_threshold,
            "collision_filter_retries": collision_filter_retries,
            "sam_select_service_name": sam_select_service_name,
        }.items(),
    )

    vlm_prompt_node = Node(
        package="ur5_manipulation",
        executable="VLM_prompt.py",
        name="vlm_sam2_prompt_node",
        output="screen",
        arguments=["--ros-node"],
        parameters=[
            {
                "instruction": vlm_instruction,
                "image_topic": "/camera/camera/color/image_raw",
                "prompt_topic": "/sam2/vlm_prompt",
                "gripper_config_topic": "/ur5/gripper/config",
                "select_service": "/vlm_prompt/select_object",
                "set_instruction_service": "/vlm_prompt/set_instruction",
                "sam_apply_service": "/sam2/apply_vlm_prompt",
                "output_dir": "/tmp/vlm_prompt",
                "save_debug_outputs": True,
                "publish_prompt_topic": False,
                # El BT recarga SAM2 y llama /sam2/apply_vlm_prompt tras liberar
                # el VLM de la GPU; la inferencia VLM no debe aplicarlo.
                "call_sam_apply_service": False,
                "vlm_server_url": vlm_server_url,
                "vlm_request_timeout_sec": vlm_request_timeout_sec,
                "sam_apply_timeout_sec": sam_apply_timeout_sec,
            }
        ],
        condition=IfCondition(enable_vlm_prompt),
    )

    point_cloud_scanner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(scanner_launch),
    )

    wait_for_driver = ExecuteProcess(
        cmd=[
            "python3",
            wait_for_services,
            "--services",
            driver_ready_services,
            "--timeout",
            readiness_timeout,
            "--label",
            "UR5 driver/MoveIt stack",
        ],
        output="screen",
    )

    wait_for_bt = ExecuteProcess(
        cmd=[
            "python3",
            wait_for_services,
            "--services",
            bt_ready_services,
            "--timeout",
            readiness_timeout,
            "--label",
            "BT grasp stack",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur5"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("robot_ip", default_value="192.168.56.101"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("tty_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument(
                "warehouse_sqlite_path",
                default_value="/root/.ros/warehouse_ros.sqlite",
                description="Path where the warehouse database should be stored.",
            ),
            DeclareLaunchArgument(
                "kinematics_params_file",
                default_value="/workspace/ur_ws/ur5_calibration.yaml",
                description="UR calibration kinematics yaml.",
            ),
            DeclareLaunchArgument(
                "gripper_config",
                default_value="/workspace/GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda.yml",
                description="Path to the GraspGen gripper configuration yaml.",
            ),
            DeclareLaunchArgument(
                "grasp_threshold",
                default_value="0.6",
                description="Minimum GraspGen score threshold.",
            ),
            DeclareLaunchArgument(
                "num_grasps",
                default_value="200",
                description="Number of grasps sampled by GraspGen.",
            ),
            DeclareLaunchArgument(
                "max_retries",
                default_value="3",
                description="Maximum number of Behavior Tree retries.",
            ),
            DeclareLaunchArgument(
                "max_grasps_to_try",
                default_value="10",
                description="Maximum number of GraspGen candidates evaluated by MoveIt.",
            ),
            DeclareLaunchArgument(
                "max_graspgen_retries",
                default_value="3",
                description="Maximum fresh GraspGen retries after IK/approach rejection.",
            ),
            DeclareLaunchArgument(
                "collision_threshold",
                default_value="0.002",
                description="Gripper-scene collision distance threshold in meters.",
            ),
            DeclareLaunchArgument(
                "collision_filter_retries",
                default_value="2",
                description="Collision-filter retries with a progressively relaxed threshold.",
            ),
            DeclareLaunchArgument(
                "readiness_timeout",
                default_value="180.0",
                description="Maximum seconds to wait for each stack readiness check.",
            ),
            DeclareLaunchArgument(
                "driver_ready_services",
                default_value="/compute_cartesian_path",
                description="Comma-separated services that indicate the driver/MoveIt stack is ready.",
            ),
            DeclareLaunchArgument(
                "bt_ready_services",
                default_value="/ur5/go_home,/ur5/run_bt_grasp_cycle,/ur5/clear_grasp_cache,/graspgen/run_inference,/graspgen/run_inference_sam,/ur5/gripper/open,/ur5/gripper/close",
                description="Comma-separated services that indicate the BT grasp stack is ready.",
            ),
            DeclareLaunchArgument(
                "bt_settle_delay",
                default_value="3.0",
                description="Seconds to wait after BT readiness so the planning scene table can be published.",
            ),
            DeclareLaunchArgument(
                "enable_vlm_prompt",
                default_value="true",
                description="Start the VLM prompt node that sends semantic prompts to SAM2.",
            ),
            DeclareLaunchArgument(
                "vlm_instruction",
                default_value="Selecciona el objeto solicitado para agarrarlo.",
                description="Instruction used by the VLM when the BT asks SAM2 to select an object.",
            ),
            DeclareLaunchArgument(
                "vlm_server_url",
                default_value="http://172.17.0.2:8888/infer_image",
                description="HTTP endpoint used by the VLM prompt node.",
            ),
            DeclareLaunchArgument(
                "vlm_request_timeout_sec",
                default_value="180.0",
                description="Maximum seconds to wait for the VLM HTTP inference response.",
            ),
            DeclareLaunchArgument(
                "sam_apply_timeout_sec",
                default_value="120.0",
                description="Maximum seconds to wait for SAM2 to apply the VLM prompt.",
            ),
            DeclareLaunchArgument(
                "sam_select_service_name",
                default_value="/sam2/select_object",
                description="Selection service used by /ur5/prepare_bt_grasp_cycle (manual SAM2 box). "
                "The VLM path uses /ur5/prepare_vlm_bt_grasp_cycle with /vlm_prompt/select_object.",
            ),
            driver,
            vlm_prompt_node,
            wait_for_driver,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_driver,
                    on_exit=[
                        bt_grasp_system,
                        wait_for_bt,
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_bt,
                    on_exit=[
                        TimerAction(
                            period=bt_settle_delay,
                            actions=[point_cloud_scanner],
                        )
                    ],
                )
            ),
        ]
    )
