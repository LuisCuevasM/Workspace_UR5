from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    # ---- Args ----
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    tty_port = LaunchConfiguration("tty_port")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")


    declare_args = [
        DeclareLaunchArgument("ur_type", default_value="ur5"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.56.101"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("tty_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value="/root/.ros/warehouse_ros.sqlite",
            description="Path where the warehouse database should be stored",
        ),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value="/workspace/ur_ws/ur5_calibration.yaml",
            description="UR calibration kinematics yaml (from ur_calibration calibration_correction)",
        ),
    ]

    # ---- Locate included launch files ----
    ur_robot_driver_share = get_package_share_directory("ur_robot_driver")
    ur_moveit_share = get_package_share_directory("ur5_moveit_config")
    robotiq_share = get_package_share_directory("robotiq_hande_driver")
    realsense_share = get_package_share_directory("realsense2_camera")

    ur_control_launch = os.path.join(
        ur_robot_driver_share, "launch", "ur_control.launch.py"
    )
    ur_moveit_launch = os.path.join(
        ur_moveit_share, "launch", "move_group.launch.py"
    )
    gripper_launch = os.path.join(
        robotiq_share, "bringup", "launch", "gripper_controller_preview.launch.py"
    )

    realsense_launch = os.path.join(
    realsense_share, "launch", "rs_launch.py"
    )

    # ---- Include: UR driver ----
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_control_launch),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "kinematics_params_file": kinematics_params_file,
        }.items(),
    )

    # ---- Include: Hand-E driver (Modbus RTU) ----
    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gripper_launch),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
            "tty_port": tty_port,
            "tf_prefix": "hande_",
        }.items(),
    )

    # ---- Include: MoveIt ----
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_moveit_launch),
    )

    # ---- Transformación Estática  ----
    # Une el 'tool0' del UR5 con el acople del Hand-E. 
    # Los argumentos son: [x, y, z, yaw, pitch, roll, frame_padre, frame_hijo]
    tf_ur5_to_gripper = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="ur5_to_hande_tf",
        arguments=[
            "0", "0", "0", "0", "0", "0", 
            "tool0", 
            "hande_robotiq_hande_link" # Ajusta este nombre si el eslabón base es hande_robotiq_hande_coupler
        ],
        output="screen"
    )

    # ---- Transformación Estática: Robot -> Cámara ----
    # Une la base del UR5 con la cámara.
    # Los argumentos son: [x, y, z, yaw, pitch, roll, frame_padre, frame_hijo]
    # x, y, z están en metros. yaw, pitch, roll están en radianes.
    tf_base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_tf",
        arguments=[
            "0.016", "-0.0485", "0.137",  # TRASLACIÓN
            "0.0", "0.0", "0.0",  # ROTACIÓN
            "tool0",          # Origen (El robot)
            "camera_link"         # Destino (La base de la cámara)
        ],
        output="screen"
    )


    gripper_joint_filler = Node(
        package="ur5_description",          # el paquete donde pusiste el script
        executable="gripper_jointstate.py",
        name="gripper_jointstate",
        output="screen",
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            "pointcloud.enable": "true",
            "align_depth.enable": "true",
            "depth_module.depth_profile": "640x480x15",  # Bajamos de 30 a 15 FPS
            "rgb_camera.color_profile": "640x480x15",   # Bajamos resolución y FPS
            "enable_sync": "true",                      # Ayuda a que los flujos no se desfasen
        }.items(),
    )

    # ---- Timing ----
    actions = [
        #TimerAction(period=2.0, actions=[ur_control]),
        TimerAction(period=4.0, actions=[gripper]),
        TimerAction(period=6.0, actions=[moveit]),
        realsense,
        #tf_base_to_camera,
        gripper_joint_filler,
    ]

    return LaunchDescription(declare_args + actions)
