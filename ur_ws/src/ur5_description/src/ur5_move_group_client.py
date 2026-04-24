#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
    PlanningSceneWorld,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Empty, Trigger
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener


class MinimalGraspExecutor(Node):
    def __init__(self):
        super().__init__("minimal_grasp_executor")

        self.group_name = self.declare_parameter("group_name", "ur_arm").value
        self.planning_pipeline = self.declare_parameter(
            "planning_pipeline", ""
        ).value
        self.planner_id = self.declare_parameter("planner_id", "").value
        self.target_frame = self.declare_parameter("target_frame", "base_link").value
        self.execute = self.declare_parameter("execute", True).value
        self.auto_start = self.declare_parameter("auto_start", True).value
        self.use_approach_stage = self.declare_parameter(
            "use_approach_stage", True
        ).value
        self.approach_offset = float(
            self.declare_parameter("approach_offset", 0.08).value
        )
        self.approach_axis = self.declare_parameter("approach_axis", "z").value
        self.approach_sign = float(
            self.declare_parameter("approach_sign", -1.0).value
        )
        self.approach_enable_replanning = bool(
            self.declare_parameter("approach_enable_replanning", True).value
        )
        self.approach_replan_attempts = int(
            self.declare_parameter("approach_replan_attempts", 5).value
        )
        self.approach_replan_delay = float(
            self.declare_parameter("approach_replan_delay", 0.5).value
        )
        self.use_cartesian_for_grasp_final = bool(
            self.declare_parameter("use_cartesian_for_grasp_final", True).value
        )
        self.cartesian_path_service_name = self.declare_parameter(
            "cartesian_path_service_name", "/compute_cartesian_path"
        ).value
        self.cartesian_max_step = float(
            self.declare_parameter("cartesian_max_step", 0.005).value
        )
        self.cartesian_jump_threshold = float(
            self.declare_parameter("cartesian_jump_threshold", 0.0).value
        )
        self.cartesian_revolute_jump_threshold = float(
            self.declare_parameter("cartesian_revolute_jump_threshold", 0.0).value
        )
        self.cartesian_prismatic_jump_threshold = float(
            self.declare_parameter("cartesian_prismatic_jump_threshold", 0.0).value
        )
        self.cartesian_min_fraction = float(
            self.declare_parameter("cartesian_min_fraction", 0.999).value
        )
        self.grasp_final_extra_advance = float(
            self.declare_parameter("grasp_final_extra_advance", 0.0).value
        )
        self.ik_link = self.declare_parameter("ik_link", "robotiq_hande_end").value
        self.grasp_pose_link = self.declare_parameter(
            "grasp_pose_link", self.ik_link
        ).value
        self.grasp_service_name = self.declare_parameter(
            "grasp_service_name", "/graspgen/run_inference"
        ).value
        self.sam_select_service_name = self.declare_parameter(
            "sam_select_service_name", "/sam2/select_object"
        ).value
        self.sam_pause_service_name = self.declare_parameter(
            "sam_pause_service_name", "/sam2/pause_inference"
        ).value
        self.sam_resume_service_name = self.declare_parameter(
            "sam_resume_service_name", "/sam2/resume_inference"
        ).value
        self.run_cycle_service_name = self.declare_parameter(
            "run_cycle_service_name", "/ur5/run_grasp_cycle"
        ).value
        self.execute_cached_cycle_service_name = self.declare_parameter(
            "execute_cached_cycle_service_name", "/ur5/execute_cached_grasp_cycle"
        ).value
        self.go_home_service_name = self.declare_parameter(
            "go_home_service_name", "/ur5/go_home"
        ).value
        self.ik_service_name = self.declare_parameter(
            "ik_service_name", "/compute_ik"
        ).value
        self.grasp_result_timeout_sec = float(
            self.declare_parameter("grasp_result_timeout_sec", 20.0).value
        )
        self.post_sam_selection_wait_sec = float(
            self.declare_parameter("post_sam_selection_wait_sec", 1.0).value
        )
        self.top_grasps_topic = self.declare_parameter(
            "top_grasps_topic", "/graspgen/top_grasps"
        ).value
        self.max_ik_retries = int(
            self.declare_parameter("max_ik_retries", 5).value
        )
        self.enable_octomap_freeze = bool(
            self.declare_parameter("enable_octomap_freeze", False).value
        )
        self.pause_sam_during_pick = bool(
            self.declare_parameter("pause_sam_during_pick", True).value
        )
        self.move_group_node_name = self.declare_parameter(
            "move_group_node_name", "/move_group"
        ).value
        self.octomap_sensor_namespace = self.declare_parameter(
            "octomap_sensor_namespace", "default_sensor"
        ).value
        self.frozen_octomap_update_rate = float(
            self.declare_parameter("frozen_octomap_update_rate", 1.0e-6).value
        )
        self.gripper_action_name = self.declare_parameter(
            "gripper_action_name",
            "gripper/gripper_action_controller/gripper_cmd",
        ).value
        self.close_gripper_position = float(
            self.declare_parameter("close_gripper_position", 0.0).value
        )
        self.close_gripper_max_effort = float(
            self.declare_parameter("close_gripper_max_effort", 0.0).value
        )
        self.open_gripper_position = float(
            self.declare_parameter("open_gripper_position", 0.025).value
        )
        self.open_gripper_max_effort = float(
            self.declare_parameter("open_gripper_max_effort", 0.0).value
        )
        self.post_grasp_lift_z = float(
            self.declare_parameter("post_grasp_lift_z", 0.10).value
        )
        self.arm_joint_names = list(
            self.declare_parameter(
                "arm_joint_names",
                [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
            ).value
        )
        self.place_joint_positions = list(
            self.declare_parameter(
                "place_joint_positions",
                [1.5010, -2.4609, -0.6981, -1.5708, 1.5708, 0.8552],
            ).value
        )
        self.home_joint_positions = list(
            self.declare_parameter(
                "home_joint_positions",
                [0.872, -1.0995, -2.1118, -1.04719, 1.5009, 0.0],
            ).value
        )

        self.min_target_z = 0.01
        self.goal_joint_tolerance = 1e-3
        self.ik_timeout = Duration(sec=0, nanosec=250_000_000)
        self.latest_arm_joint_state = None
        self.latest_best_grasp_msg = None
        self.latest_best_grasp_arrival_ns = 0
        self.latest_top_grasps_msg = None
        self.latest_top_grasps_arrival_ns = 0
        self.awaiting_grasp_result = False
        self.request_in_flight = False
        self.cycle_active = False
        self.current_request_id = 0
        self.current_request_start_ns = 0
        self.current_request_reason = ""
        self.current_retry_count = 0
        self.start_cycle_requested = False
        self.current_motion_phase = "idle"
        self.current_gripper_phase = "idle"
        self.gripper_goal_active = False
        self.pending_grasp_goal_state = None
        self.pending_grasp_target_pose = None
        self.pending_motion_log_label = ""
        self.current_motion_uses_move_group_execution = False
        self.octomap_frozen = False
        self.sam_paused_for_pick = False
        self.octomap_restore_rate = None
        self.cycle_done_event = threading.Event()
        self.last_cycle_succeeded = False
        self.last_cycle_result_message = "Sin ejecucion previa"
        self.moveit_cb_group = ReentrantCallbackGroup()
        self.subscriptions_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = ReentrantCallbackGroup()

        if len(self.place_joint_positions) != len(self.arm_joint_names):
            raise RuntimeError(
                "place_joint_positions y arm_joint_names deben tener el mismo largo"
            )
        if len(self.home_joint_positions) != len(self.arm_joint_names):
            raise RuntimeError(
                "home_joint_positions y arm_joint_names deben tener el mismo largo"
            )

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MoveIt clients
        self.action_client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
            callback_group=self.moveit_cb_group,
        )
        self.ik_client = self.create_client(
            GetPositionIK,
            self.ik_service_name,
            callback_group=self.moveit_cb_group,
        )
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            self.cartesian_path_service_name,
            callback_group=self.moveit_cb_group,
        )
        self.grasp_client = self.create_client(
            Trigger,
            self.grasp_service_name,
            callback_group=self.moveit_cb_group,
        )
        self.sam_select_client = self.create_client(
            Trigger,
            self.sam_select_service_name,
            callback_group=self.moveit_cb_group,
        )
        self.sam_pause_client = self.create_client(
            Trigger,
            self.sam_pause_service_name,
            callback_group=self.moveit_cb_group,
        )
        self.sam_resume_client = self.create_client(
            Trigger,
            self.sam_resume_service_name,
            callback_group=self.moveit_cb_group,
        )
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            self.gripper_action_name,
            callback_group=self.moveit_cb_group,
        )
        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            "/execute_trajectory",
            callback_group=self.moveit_cb_group,
        )
        self.clear_octomap_client = self.create_client(
            Empty,
            "/clear_octomap",
            callback_group=self.moveit_cb_group,
        )
        self.get_move_group_params_client = self.create_client(
            GetParameters,
            f"{self.move_group_node_name}/get_parameters",
            callback_group=self.moveit_cb_group,
        )
        self.set_move_group_params_client = self.create_client(
            SetParameters,
            f"{self.move_group_node_name}/set_parameters",
            callback_group=self.moveit_cb_group,
        )

        # Planning scene publisher
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.run_cycle_service = self.create_service(
            Trigger,
            self.run_cycle_service_name,
            self._handle_run_cycle_request,
            callback_group=self.moveit_cb_group,
        )
        self.execute_cached_cycle_service = self.create_service(
            Trigger,
            self.execute_cached_cycle_service_name,
            self._handle_execute_cached_cycle_request,
            callback_group=self.moveit_cb_group,
        )
        self.go_home_service = self.create_service(
            Trigger,
            self.go_home_service_name,
            self._handle_go_home_request,
            callback_group=self.moveit_cb_group,
        )

        # Subscribers
        self.create_subscription(
            PoseStamped,
            "/graspgen/best_grasp",
            self._on_best_grasp,
            10,
            callback_group=self.subscriptions_cb_group,
        )
        self.create_subscription(
            PoseArray,
            self.top_grasps_topic,
            self._on_top_grasps,
            10,
            callback_group=self.subscriptions_cb_group,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            50,
            callback_group=self.subscriptions_cb_group,
        )

        # Publicar mesa una vez
        self.table_timer = self.create_timer(
            2.0, self._publish_table, callback_group=self.timer_cb_group
        )
        self.workflow_timer = self.create_timer(
            0.2, self._workflow_tick, callback_group=self.timer_cb_group
        )

        self.get_logger().info(
            "Nodo listo. Coordina approach -> grasp -> place -> home "
            f"usando link objetivo '{self.ik_link}' "
            f"(grasp_pose_link='{self.grasp_pose_link}', execute={self.execute}, "
            f"auto_start={self.auto_start}, planning_pipeline='{self.planning_pipeline}', "
            f"planner_id='{self.planner_id}', use_approach_stage={self.use_approach_stage}, "
            f"approach_axis='{self.approach_axis}', approach_offset={self.approach_offset:.3f}, "
            f"approach_sign={self.approach_sign:.1f}, "
            f"approach_enable_replanning={self.approach_enable_replanning}, "
            f"approach_replan_attempts={self.approach_replan_attempts}, "
            f"approach_replan_delay={self.approach_replan_delay:.2f}, "
            f"use_cartesian_for_grasp_final={self.use_cartesian_for_grasp_final}, "
            f"cartesian_max_step={self.cartesian_max_step:.4f}, "
            f"cartesian_min_fraction={self.cartesian_min_fraction:.3f}, "
            f"grasp_final_extra_advance={self.grasp_final_extra_advance:.4f}, "
            f"top_grasps_topic='{self.top_grasps_topic}', "
            f"enable_octomap_freeze={self.enable_octomap_freeze}, "
            f"pause_sam_during_pick={self.pause_sam_during_pick}, "
            f"sam_select_service='{self.sam_select_service_name}')."
        )

    def _wait_for_future_result(self, future, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)

        if not future.done():
            return None
        return future.result()

    def _octomap_update_rate_param(self) -> str:
        return f"{self.octomap_sensor_namespace}.max_update_rate"

    def _set_octomap_update_rate(self, rate_hz: float) -> bool:
        param_name = self._octomap_update_rate_param()

        if not self.set_move_group_params_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f"No se pudo contactar con el nodo de parametros '{self.move_group_node_name}' "
                "para actualizar el octomap."
            )
            return False

        request = SetParameters.Request()
        request.parameters = [
            ParameterMsg(
                name=param_name,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(rate_hz),
                ),
            )
        ]

        future = self.set_move_group_params_client.call_async(request)
        result = self._wait_for_future_result(future, timeout_sec=2.0)
        if not result:
            self.get_logger().warn(
                f"Timeout ajustando el parametro '{param_name}' a {rate_hz} Hz."
            )
            return False

        if not result.results or not result.results[0].successful:
            self.get_logger().warn(
                f"No se pudo fijar '{param_name}'={rate_hz}: "
                f"{result.results[0].reason if result.results else 'sin detalle'}"
            )
            return False

        return True

    def _freeze_octomap(self, trigger_label: str):
        if not self.enable_octomap_freeze or self.octomap_frozen:
            if self.enable_octomap_freeze and self.octomap_frozen:
                self.get_logger().info(
                    f"Octomap ya estaba congelado; se mantiene asi durante {trigger_label}."
                )
            return

        param_name = self._octomap_update_rate_param()
        if self.octomap_restore_rate is None:
            if not self.get_move_group_params_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(
                    f"No se pudo contactar con el nodo de parametros '{self.move_group_node_name}' "
                    "para leer el octomap."
                )
                return

            request = GetParameters.Request()
            request.names = [param_name]
            future = self.get_move_group_params_client.call_async(request)
            values = self._wait_for_future_result(future, timeout_sec=2.0)
            if not values or not values.values:
                self.get_logger().warn(
                    "No pude leer la tasa original del octomap; se omite freeze."
                )
                return

            current_value = values.values[0].double_value
            if current_value <= 0.0:
                self.get_logger().warn(
                    f"Valor actual invalido para '{param_name}': {current_value}."
                )
                return
            self.octomap_restore_rate = current_value

        if self._set_octomap_update_rate(self.frozen_octomap_update_rate):
            self.octomap_frozen = True
            self.get_logger().info(
                f"Octomap congelado tras {trigger_label}. "
                f"{param_name}: {self.octomap_restore_rate} -> {self.frozen_octomap_update_rate} Hz."
            )

    def _restore_octomap(self):
        if not self.octomap_frozen or self.octomap_restore_rate is None:
            return

        param_name = self._octomap_update_rate_param()
        if self._set_octomap_update_rate(self.octomap_restore_rate):
            self.get_logger().info(
                f"Octomap restaurado. {param_name}: "
                f"{self.frozen_octomap_update_rate} -> {self.octomap_restore_rate} Hz."
            )
            self.octomap_frozen = False

    def _should_freeze_octomap_for_phase(self, phase: str) -> bool:
        return self.enable_octomap_freeze and phase in ("approach", "grasp")

    def _call_trigger_and_wait(
        self, client, service_name: str, timeout_sec: float, action_label: str
    ) -> bool:
        if not service_name:
            return False

        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f"Servicio '{service_name}' no disponible para {action_label}."
            )
            return False

        future = client.call_async(Trigger.Request())
        response = self._wait_for_future_result(future, timeout_sec=timeout_sec)
        if response is None:
            self.get_logger().warn(
                f"Timeout llamando a '{service_name}' para {action_label}."
            )
            return False

        if not response.success:
            self.get_logger().warn(
                f"Servicio '{service_name}' fallo durante {action_label}: "
                f"{response.message}"
            )
            return False

        return True

    def _pause_sam_for_pick(self, trigger_label: str):
        if not self.pause_sam_during_pick or self.sam_paused_for_pick:
            if self.pause_sam_during_pick and self.sam_paused_for_pick:
                self.get_logger().info(
                    f"SAM ya estaba pausado; se mantiene asi durante {trigger_label}."
                )
            return

        if self._call_trigger_and_wait(
            self.sam_pause_client,
            self.sam_pause_service_name,
            timeout_sec=2.0,
            action_label=f"pausar SAM durante {trigger_label}",
        ):
            self.sam_paused_for_pick = True
            self.get_logger().info(
                f"Inferencia SAM pausada durante {trigger_label}."
            )

    def _resume_sam_after_pick(self, trigger_label: str):
        if not self.sam_paused_for_pick:
            return

        if self._call_trigger_and_wait(
            self.sam_resume_client,
            self.sam_resume_service_name,
            timeout_sec=2.0,
            action_label=f"reanudar SAM tras {trigger_label}",
        ):
            self.sam_paused_for_pick = False
            self.get_logger().info(
                f"Inferencia SAM reanudada tras {trigger_label}."
            )

    def _clear_octomap(self, trigger_label: str):
        if not self.clear_octomap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "Servicio '/clear_octomap' no disponible; se omite limpieza del octomap."
            )
            return

        future = self.clear_octomap_client.call_async(Empty.Request())
        response = self._wait_for_future_result(future, timeout_sec=2.0)
        if response is None:
            self.get_logger().warn(
                f"Timeout limpiando octomap tras {trigger_label}."
            )
            return

        self.get_logger().info(f"Octomap limpiado tras {trigger_label}.")

    def _publish_table(self):
        co = CollisionObject(id="table", operation=CollisionObject.ADD)
        co.header.frame_id = self.target_frame

        co.primitives.append(
            SolidPrimitive(
                type=SolidPrimitive.BOX,
                dimensions=[1.10, 1.10, 0.05],
            )
        )

        p = Pose()
        p.position.z = -0.03
        p.position.x = -0.19
        p.position.y = -0.3
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.3894
        p.orientation.w = 0.9211

        co.primitive_poses.append(p)

        world = PlanningSceneWorld()
        world.collision_objects.append(co)

        self.scene_pub.publish(PlanningScene(world=world, is_diff=True))
        self.table_timer.cancel()

    def _on_joint_states(self, msg: JointState):
        joint_map = {
            name: position for name, position in zip(msg.name, msg.position)
        }

        if not all(name in joint_map for name in self.arm_joint_names):
            return

        arm_state = JointState()
        arm_state.header = msg.header
        arm_state.name = list(self.arm_joint_names)
        arm_state.position = [joint_map[name] for name in self.arm_joint_names]
        self.latest_arm_joint_state = arm_state

    def _msg_stamp_to_ns(self, stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _workflow_tick(self):
        if (
            self.auto_start
            and not self.start_cycle_requested
            and not self.cycle_active
            and self.latest_arm_joint_state is not None
        ):
            self.start_cycle_requested = True
            self._start_new_cycle("inicio")

        if self.awaiting_grasp_result:
            waited_sec = (
                self.get_clock().now().nanoseconds - self.current_request_start_ns
            ) / 1e9
            if waited_sec > self.grasp_result_timeout_sec:
                self.awaiting_grasp_result = False
                self.get_logger().warn(
                    f"Timeout esperando grasps candidatos para request_id={self.current_request_id}."
                )
                self._retry_grasp("timeout esperando grasp")

    def _request_new_grasp(self, reason: str):
        if self.request_in_flight:
            self.get_logger().warn(
                "Ya hay una peticion de grasp en curso; se omite una nueva solicitud."
            )
            return

        if not self.grasp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                f"Servicio de grasp '{self.grasp_service_name}' no disponible."
            )
            return

        self.current_request_id += 1
        self.current_request_reason = reason
        self.current_request_start_ns = self.get_clock().now().nanoseconds
        self.awaiting_grasp_result = True
        self.request_in_flight = True

        self.get_logger().info(
            f"Solicitando nuevo grasp a {self.grasp_service_name} "
            f"(request_id={self.current_request_id}, reason='{reason}', retry={self.current_retry_count})."
        )

        future = self.grasp_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_grasp_service_response)

    def _request_sam_selection(self, reason: str) -> bool:
        if not self.sam_select_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                f"Servicio SAM '{self.sam_select_service_name}' no disponible."
            )
            return False

        self.get_logger().info(
            f"Solicitando seleccion de objeto a {self.sam_select_service_name} "
            f"antes del grasp ({reason})."
        )
        future = self.sam_select_client.call_async(Trigger.Request())
        response = self._wait_for_future_result(future, timeout_sec=120.0)
        if response is None:
            self.get_logger().error(
                "Timeout esperando la seleccion de objeto de SAM."
            )
            return False

        if not response.success:
            self.get_logger().warn(
                f"Seleccion SAM cancelada o fallida: {response.message}"
            )
            return False

        if self.post_sam_selection_wait_sec > 0.0:
            self.get_logger().info(
                f"Esperando {self.post_sam_selection_wait_sec:.1f} s para que SAM publique nubes."
            )
            time.sleep(self.post_sam_selection_wait_sec)

        return True

    def _start_new_cycle(self, reason: str):
        if self.cycle_active:
            self.get_logger().warn("Ya hay un ciclo grasp->IK->MoveIt en ejecucion.")
            return False

        if self.latest_arm_joint_state is None:
            self.get_logger().warn(
                "No se puede iniciar el ciclo aun: faltan /joint_states del brazo."
            )
            return False

        self._prepare_cycle_start()
        if not self._request_sam_selection(reason):
            self.cycle_active = False
            return False
        self._request_new_grasp(reason)
        return True

    def _prepare_cycle_start(self):
        self.cycle_active = True
        self.current_retry_count = 0
        self.last_cycle_succeeded = False
        self.last_cycle_result_message = "Ciclo en ejecucion"
        self.cycle_done_event.clear()

    def _start_cycle_from_cached_grasps(self, reason: str) -> bool:
        if self.cycle_active:
            self.get_logger().warn("Ya hay un ciclo grasp->IK->MoveIt en ejecucion.")
            return False

        if self.latest_arm_joint_state is None:
            self.get_logger().warn(
                "No se puede iniciar el ciclo desde grasps cacheados: faltan /joint_states del brazo."
            )
            return False

        if (
            self.latest_top_grasps_msg is not None
            and len(self.latest_top_grasps_msg.poses) > 0
        ):
            self._prepare_cycle_start()
            self.get_logger().info(
                f"Iniciando ciclo desde grasps cacheados via top_grasps ({reason})."
            )
            self._process_grasp_candidates(
                self.latest_top_grasps_msg.header.frame_id,
                self.latest_top_grasps_msg.poses,
                source_label="top_grasps_cached",
                retry_on_failure=False,
            )
            return True

        if self.latest_best_grasp_msg is not None:
            self._prepare_cycle_start()
            self.get_logger().info(
                f"Iniciando ciclo desde grasp cacheado via best_grasp ({reason})."
            )
            self._process_grasp_candidates(
                self.latest_best_grasp_msg.header.frame_id,
                [self.latest_best_grasp_msg.pose],
                source_label="best_grasp_cached",
                retry_on_failure=False,
            )
            return True

        self.get_logger().warn(
            "No hay grasps cacheados disponibles para ejecutar desde el orquestador externo."
        )
        return False

    def _finish_cycle(self, success: bool = False, message: str = ""):
        self._restore_octomap()
        self.cycle_active = False
        self.awaiting_grasp_result = False
        self.request_in_flight = False
        self.current_motion_phase = "idle"
        self.current_gripper_phase = "idle"
        self.gripper_goal_active = False
        self.pending_grasp_goal_state = None
        self.pending_grasp_target_pose = None
        self.pending_motion_log_label = ""
        self.current_motion_uses_move_group_execution = False
        self.last_cycle_succeeded = success
        self.last_cycle_result_message = message or (
            "Ciclo completado correctamente"
            if success
            else "Ciclo finalizado con error o cancelacion"
        )
        self.cycle_done_event.set()

    def _handle_run_cycle_request(self, request, response):
        del request

        if self._start_new_cycle("servicio run_grasp_cycle"):
            response.success = True
            response.message = "Ciclo grasp->IK->MoveIt iniciado"
        else:
            response.success = False
            if self.cycle_active:
                response.message = "Ya hay un ciclo en ejecucion"
            elif self.latest_arm_joint_state is None:
                response.message = "Faltan /joint_states del brazo"
            else:
                response.message = "No se pudo iniciar el ciclo"
        return response

    def _handle_execute_cached_cycle_request(self, request, response):
        del request

        if not self._start_cycle_from_cached_grasps(
            "servicio execute_cached_grasp_cycle"
        ):
            response.success = False
            if self.cycle_active:
                response.message = "Ya hay un ciclo en ejecucion"
            elif self.latest_arm_joint_state is None:
                response.message = "Faltan /joint_states del brazo"
            else:
                response.message = "No hay grasps cacheados disponibles"
            return response

        completed = self.cycle_done_event.wait(timeout=self.grasp_result_timeout_sec + 60.0)
        if not completed:
            response.success = False
            response.message = "Timeout esperando que termine el ciclo cacheado"
            return response

        response.success = self.last_cycle_succeeded
        response.message = self.last_cycle_result_message
        return response

    def _handle_go_home_request(self, request, response):
        del request

        if self.cycle_active:
            response.success = False
            response.message = "Ya hay un ciclo o movimiento en ejecucion"
            return response

        if self.latest_arm_joint_state is None:
            response.success = False
            response.message = "Faltan /joint_states del brazo"
            return response

        self._prepare_cycle_start()
        self.get_logger().info("Iniciando retorno a home solicitado por servicio.")
        self._send_joint_goal(
            self._joint_state_from_positions(self.home_joint_positions),
            "home",
            "home solicitado",
        )

        completed = self.cycle_done_event.wait(timeout=90.0)
        if not completed:
            response.success = False
            response.message = "Timeout esperando retorno a home"
            return response

        response.success = self.last_cycle_succeeded
        response.message = self.last_cycle_result_message
        return response

    def _on_grasp_service_response(self, future):
        self.request_in_flight = False
        try:
            response = future.result()
            if response is None:
                self.awaiting_grasp_result = False
                self.get_logger().error("Servicio de grasp no devolvio respuesta.")
                self._retry_grasp("sin respuesta del servicio grasp")
                return

            if not response.success:
                self.awaiting_grasp_result = False
                self.get_logger().warn(
                    f"Servicio de grasp fallo: {response.message}"
                )
                self._retry_grasp(f"servicio grasp fallo: {response.message}")
                return

            self.get_logger().info(
                f"Servicio de grasp completado. Esperando grasps frescos "
                f"(request_id={self.current_request_id})."
            )

            # Los grasps pueden haber llegado antes de la respuesta del servicio.
            if (
                self.latest_top_grasps_msg is not None
                and self.latest_top_grasps_arrival_ns >= self.current_request_start_ns
                and self.awaiting_grasp_result
                and len(self.latest_top_grasps_msg.poses) > 0
            ):
                self.awaiting_grasp_result = False
                self._process_grasp_candidates(
                    self.latest_top_grasps_msg.header.frame_id,
                    self.latest_top_grasps_msg.poses,
                    source_label="top_grasps",
                )
                return

            if (
                self.latest_best_grasp_msg is not None
                and self.latest_best_grasp_arrival_ns >= self.current_request_start_ns
                and self.awaiting_grasp_result
            ):
                self.awaiting_grasp_result = False
                self._process_grasp_candidates(
                    self.latest_best_grasp_msg.header.frame_id,
                    [self.latest_best_grasp_msg.pose],
                    source_label="best_grasp_fallback",
                )
        except Exception as e:
            self.awaiting_grasp_result = False
            self.get_logger().error(f"Error en respuesta del servicio grasp: {e}")
            self._retry_grasp("excepcion en servicio grasp")

    def _retry_grasp(self, reason: str):
        if self.current_retry_count >= self.max_ik_retries:
            self.get_logger().error(
                f"Se agotaron los reintentos de grasp/IK ({self.max_ik_retries})."
            )
            self._finish_cycle()
            return

        self.current_retry_count += 1
        self._request_new_grasp(reason)

    def _is_target_reachable(self, target_pose: Pose) -> bool:
        radial_distance = (
            target_pose.position.x**2 + target_pose.position.y**2
        ) ** 0.5

        if target_pose.position.z < self.min_target_z:
            self.get_logger().warn(
                f"Objetivo descartado: z={target_pose.position.z:.3f} m demasiado bajo."
            )
            return False

        if radial_distance > 0.85:
            self.get_logger().warn(
                f"Objetivo descartado: distancia radial {radial_distance:.3f} m fuera de alcance."
            )
            return False

        return True

    def _robot_state_from_joint_state(self, joint_state: JointState) -> RobotState:
        robot_state = RobotState()
        robot_state.joint_state = JointState()
        robot_state.joint_state.header = joint_state.header
        robot_state.joint_state.name = list(joint_state.name)
        robot_state.joint_state.position = list(joint_state.position)
        robot_state.is_diff = False
        return robot_state

    def _quat_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    def _quat_conjugate(self, q):
        x, y, z, w = q
        return (-x, -y, -z, w)

    def _quat_normalize(self, q):
        norm = math.sqrt(sum(value * value for value in q))
        if norm == 0.0:
            return (0.0, 0.0, 0.0, 1.0)
        return tuple(value / norm for value in q)

    def _rotate_vector(self, quat, vector):
        q = self._quat_normalize(quat)
        v_quat = (vector[0], vector[1], vector[2], 0.0)
        rotated = self._quat_multiply(
            self._quat_multiply(q, v_quat),
            self._quat_conjugate(q),
        )
        return rotated[:3]

    def _compose_pose_with_offset(self, base_pose: Pose, offset_transform) -> Pose:
        base_quat = (
            base_pose.orientation.x,
            base_pose.orientation.y,
            base_pose.orientation.z,
            base_pose.orientation.w,
        )
        offset_quat = (
            offset_transform.rotation.x,
            offset_transform.rotation.y,
            offset_transform.rotation.z,
            offset_transform.rotation.w,
        )
        offset_xyz = (
            offset_transform.translation.x,
            offset_transform.translation.y,
            offset_transform.translation.z,
        )

        rotated_offset = self._rotate_vector(base_quat, offset_xyz)

        pose = Pose()
        pose.position.x = base_pose.position.x + rotated_offset[0]
        pose.position.y = base_pose.position.y + rotated_offset[1]
        pose.position.z = base_pose.position.z + rotated_offset[2]

        q = self._quat_normalize(self._quat_multiply(base_quat, offset_quat))
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    def _offset_pose_along_local_axis(
        self, base_pose: Pose, axis: str, distance: float
    ) -> Pose:
        axis_vectors = {
            "x": (distance, 0.0, 0.0),
            "y": (0.0, distance, 0.0),
            "z": (0.0, 0.0, distance),
        }
        if axis not in axis_vectors:
            raise ValueError(f"Eje de aproximacion invalido: {axis}")

        base_quat = (
            base_pose.orientation.x,
            base_pose.orientation.y,
            base_pose.orientation.z,
            base_pose.orientation.w,
        )
        offset_xyz = self._rotate_vector(base_quat, axis_vectors[axis])

        pose = Pose()
        pose.position.x = base_pose.position.x + offset_xyz[0]
        pose.position.y = base_pose.position.y + offset_xyz[1]
        pose.position.z = base_pose.position.z + offset_xyz[2]
        pose.orientation = base_pose.orientation
        return pose

    def _offset_pose_along_world_z(self, base_pose: Pose, distance: float) -> Pose:
        pose = Pose()
        pose.position.x = base_pose.position.x
        pose.position.y = base_pose.position.y
        pose.position.z = base_pose.position.z + distance
        pose.orientation = base_pose.orientation
        return pose

    def _convert_pose_for_ik_link(self, target_pose: Pose) -> Pose:
        if self.grasp_pose_link == self.ik_link:
            return target_pose

        transform = self.tf_buffer.lookup_transform(
            self.grasp_pose_link,
            self.ik_link,
            rclpy.time.Time(),
        )
        return self._compose_pose_with_offset(target_pose, transform.transform)

    def _build_cartesian_grasp_pose(self, target_pose: Pose) -> Pose:
        if self.grasp_final_extra_advance == 0.0:
            return target_pose

        return self._offset_pose_along_local_axis(
            target_pose,
            self.approach_axis,
            -self.approach_sign * self.grasp_final_extra_advance,
        )

    def _build_post_grasp_lift_pose(self):
        if self.pending_grasp_target_pose is None:
            self.get_logger().error(
                "No hay pose de grasp pendiente para calcular el lift post-grasp."
            )
            return None

        if self.post_grasp_lift_z <= 0.0:
            return self.pending_grasp_target_pose

        return self._offset_pose_along_world_z(
            self.pending_grasp_target_pose,
            self.post_grasp_lift_z,
        )

    def _normalize_goal_near_seed(self, goal_state: JointState, seed_state: JointState):
        normalized_positions = []
        total_distance = 0.0

        for goal, seed in zip(goal_state.position, seed_state.position):
            delta = math.atan2(math.sin(goal - seed), math.cos(goal - seed))
            nearest_goal = seed + delta
            normalized_positions.append(nearest_goal)
            total_distance += abs(delta)

        normalized = JointState()
        normalized.header = goal_state.header
        normalized.name = list(goal_state.name)
        normalized.position = normalized_positions
        return normalized, total_distance

    def _solve_ik(self, ik_pose: Pose, seed_state: JointState):
        if not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                f"Servicio IK '{self.ik_service_name}' no disponible."
            )
            return None

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.robot_state = self._robot_state_from_joint_state(seed_state)
        request.ik_request.avoid_collisions = True
        request.ik_request.ik_link_name = self.ik_link
        request.ik_request.pose_stamped.header.frame_id = self.target_frame
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = ik_pose
        request.ik_request.timeout = self.ik_timeout

        future = self.ik_client.call_async(request)
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)

        if not future.done() or future.result() is None:
            self.get_logger().error(
                "No hubo respuesta valida del servicio IK en 2.0 s."
            )
            return None

        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(
                "IK invalida para el grasp recibido "
                f"(error_code={response.error_code.val})."
            )
            return None

        goal_state = response.solution.joint_state
        joint_map = {
            name: position for name, position in zip(goal_state.name, goal_state.position)
        }
        if not all(name in joint_map for name in self.arm_joint_names):
            self.get_logger().error(
                "La solucion IK no contiene todas las articulaciones del brazo."
            )
            return None

        filtered_goal = JointState()
        filtered_goal.header = goal_state.header
        filtered_goal.name = list(self.arm_joint_names)
        filtered_goal.position = [joint_map[name] for name in self.arm_joint_names]

        return self._normalize_goal_near_seed(filtered_goal, seed_state)

    def _build_joint_goal_constraints(self, goal_state: JointState) -> Constraints:
        constraints = Constraints()
        for joint_name, joint_position in zip(goal_state.name, goal_state.position):
            constraints.joint_constraints.append(
                JointConstraint(
                    joint_name=joint_name,
                    position=joint_position,
                    tolerance_above=self.goal_joint_tolerance,
                    tolerance_below=self.goal_joint_tolerance,
                    weight=1.0,
                )
            )
        return constraints

    def _joint_state_from_positions(self, positions) -> JointState:
        joint_state = JointState()
        joint_state.name = list(self.arm_joint_names)
        joint_state.position = [float(position) for position in positions]
        return joint_state

    def _should_use_move_group_replanning(self, phase: str) -> bool:
        return self.execute and phase == "approach" and self.approach_enable_replanning

    def _advance_after_motion_success(self, phase: str):
        if phase == "approach":
            if self.pending_grasp_goal_state is None:
                self.get_logger().error(
                    "No hay objetivo de grasp final pendiente tras la aproximacion."
                )
                self._finish_cycle()
                return
            if (
                self.use_cartesian_for_grasp_final
                and self.pending_grasp_target_pose is not None
            ):
                if not self._send_cartesian_grasp_goal(
                    self.pending_grasp_target_pose,
                    "grasp final cartesiano",
                ):
                    self._finish_cycle()
                return
            self._send_joint_goal(
                self.pending_grasp_goal_state,
                "grasp",
                "grasp final",
            )
        elif phase == "grasp":
            self._send_gripper_goal(
                self.close_gripper_position,
                self.close_gripper_max_effort,
                "after_close",
                "cerrar grasp",
            )
        elif phase == "post_grasp_lift":
            self._send_joint_goal(
                self._joint_state_from_positions(self.place_joint_positions),
                "place",
                "deposito/place",
            )
        elif phase == "place":
            self._send_gripper_goal(
                self.open_gripper_position,
                self.open_gripper_max_effort,
                "after_open",
                "soltar objeto",
            )
        elif phase == "home":
            self.get_logger().info(
                "Robot en home. Verificando apertura del gripper antes de cerrar el ciclo."
            )
            self._send_gripper_goal(
                self.open_gripper_position,
                self.open_gripper_max_effort,
                "verify_open_at_home",
                "verificar gripper abierto en home",
            )
        else:
            self._finish_cycle()

    def _send_joint_goal(self, goal_joint_state: JointState, phase: str, log_label: str):
        if self.latest_arm_joint_state is None:
            self.get_logger().warn(
                f"No hay /joint_states completos del brazo; no puedo enviar goal para {log_label}."
            )
            self._finish_cycle()
            return

        goal = MoveGroup.Goal()

        req = goal.request
        req.group_name = self.group_name
        req.pipeline_id = self.planning_pipeline
        req.planner_id = self.planner_id
        req.start_state = self._robot_state_from_joint_state(
            self.latest_arm_joint_state
        )
        req.allowed_planning_time = 25.0
        req.num_planning_attempts = 10
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        req.goal_constraints.append(self._build_joint_goal_constraints(goal_joint_state))
        use_move_group_execution = self._should_use_move_group_replanning(phase)
        goal.planning_options.plan_only = not use_move_group_execution
        goal.planning_options.replan = use_move_group_execution
        if use_move_group_execution:
            goal.planning_options.replan_attempts = self.approach_replan_attempts
            goal.planning_options.replan_delay = self.approach_replan_delay

        self.current_motion_phase = phase
        self.pending_motion_log_label = log_label
        self.current_motion_uses_move_group_execution = use_move_group_execution
        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_move_group_response)

        q_goal = ", ".join(
            f"{name}={position:.3f}"
            for name, position in zip(goal_joint_state.name, goal_joint_state.position)
        )
        self.get_logger().info(
            f"Planificacion solicitada para {log_label}. "
            f"pipeline='{self.planning_pipeline or 'default'}' | "
            f"planner_id='{self.planner_id or 'default'}' | "
            f"move_group_execution={use_move_group_execution} | "
            f"replan={goal.planning_options.replan} | "
            f"q_goal=[{q_goal}]"
        )

    def _send_cartesian_pose_goal(
        self,
        target_pose: Pose,
        phase: str,
        log_label: str,
    ) -> bool:
        if self.latest_arm_joint_state is None:
            self.get_logger().error(
                f"No hay /joint_states completos del brazo para {log_label}."
            )
            return False

        if not self.cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                f"Servicio cartesiano '{self.cartesian_path_service_name}' no disponible."
            )
            return False

        request = GetCartesianPath.Request()
        request.header.frame_id = self.target_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state = self._robot_state_from_joint_state(
            self.latest_arm_joint_state
        )
        request.group_name = self.group_name
        request.link_name = self.grasp_pose_link
        request.waypoints = [target_pose]
        request.max_step = self.cartesian_max_step
        request.jump_threshold = self.cartesian_jump_threshold
        request.revolute_jump_threshold = self.cartesian_revolute_jump_threshold
        request.prismatic_jump_threshold = self.cartesian_prismatic_jump_threshold
        request.avoid_collisions = True

        future = self.cartesian_path_client.call_async(request)
        response = self._wait_for_future_result(future, timeout_sec=5.0)
        if response is None:
            self.get_logger().error(
                f"Timeout calculando path cartesiano para {log_label}."
            )
            return False

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"GetCartesianPath fallo para {log_label}. "
                f"error_code={response.error_code.val}, fraction={response.fraction:.3f}"
            )
            return False

        if response.fraction < self.cartesian_min_fraction:
            self.get_logger().error(
                f"Path cartesiano incompleto para {log_label}: "
                f"fraction={response.fraction:.3f} < {self.cartesian_min_fraction:.3f}"
            )
            return False

        self.current_motion_phase = phase
        self.pending_motion_log_label = log_label
        self.get_logger().info(
            f"Path cartesiano listo para {log_label}: "
            f"fraction={response.fraction:.3f}, max_step={self.cartesian_max_step:.4f}, "
            f"link_name='{self.grasp_pose_link}'"
        )
        self._execute_planned_trajectory(
            response.solution,
            phase,
            log_label,
        )
        return True

    def _send_cartesian_grasp_goal(self, target_pose: Pose, log_label: str) -> bool:
        return self._send_cartesian_pose_goal(target_pose, "grasp", log_label)

    def _send_gripper_goal(
        self,
        position: float,
        max_effort: float,
        next_phase: str,
        log_label: str,
    ):
        if self.gripper_goal_active:
            self.get_logger().warn("Ya hay un goal de gripper en curso.")
            return

        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(
                f"Action del gripper '{self.gripper_action_name}' no esta disponible."
            )
            self._finish_cycle()
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = float(max_effort)

        self.current_gripper_phase = next_phase
        self.gripper_goal_active = True
        self.get_logger().info(
            f"Enviando comando al gripper para {log_label}: "
            f"position={position:.4f}, max_effort={max_effort:.4f}"
        )
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._on_gripper_goal_response)

    def _on_move_group_response(self, future):
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error("MoveGroup rechazo el goal.")
                self._finish_cycle()
                return

            self.get_logger().info(
                f"Goal aceptado para planificacion en fase '{self.current_motion_phase}'."
            )
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_move_group_result)
        except Exception as e:
            self.get_logger().error(f"Error recibiendo respuesta de MoveGroup: {e}")
            self._finish_cycle()

    def _execute_planned_trajectory(self, trajectory, phase: str, log_label: str):
        if not self.execute:
            self._finish_cycle()
            return

        if self._should_freeze_octomap_for_phase(phase):
            self._freeze_octomap(f"planificacion valida de '{log_label}'")
            self._pause_sam_for_pick(f"fase '{phase}'")
            self.get_logger().info(
                f"Proteccion de percepcion activa antes de ejecutar '{log_label}': "
                f"octomap_frozen={self.octomap_frozen}, "
                f"sam_paused={self.sam_paused_for_pick}."
            )

        if not self.execute_trajectory_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(
                "Action '/execute_trajectory' no esta disponible."
            )
            self._finish_cycle()
            return

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(
            f"Ejecutando trayectoria preplanificada para {log_label} en fase '{phase}'."
        )
        send_goal_future = self.execute_trajectory_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_execute_trajectory_response)

    def _on_move_group_result(self, future):
        try:
            result_wrapper = future.result()
            if result_wrapper is None:
                self.get_logger().error("MoveGroup no devolvio resultado.")
                self._finish_cycle()
                return

            error_code = result_wrapper.result.error_code.val
            if error_code == MoveItErrorCodes.SUCCESS:
                phase = self.current_motion_phase
                log_label = self.pending_motion_log_label or phase
                if self.current_motion_uses_move_group_execution:
                    self.get_logger().info(
                        f"MoveGroup completo planificacion+ejecucion en fase '{phase}' "
                        f"para {log_label} con replanificacion activa."
                    )
                    self._advance_after_motion_success(phase)
                else:
                    self.get_logger().info(
                        f"Plan generado correctamente en fase '{phase}' para {log_label}."
                    )
                    self._execute_planned_trajectory(
                        result_wrapper.result.planned_trajectory,
                        phase,
                        log_label,
                    )
            else:
                self.get_logger().error(
                    f"Fallo al planificar la trayectoria en fase '{self.current_motion_phase}'. "
                    f"error_code={error_code}"
                )
                self._finish_cycle()
        except Exception as e:
            self.get_logger().error(f"Error procesando resultado de MoveGroup: {e}")
            self._finish_cycle()

    def _on_execute_trajectory_response(self, future):
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error(
                    f"ExecuteTrajectory rechazo el goal en fase '{self.current_motion_phase}'."
                )
                self._finish_cycle()
                return

            self.get_logger().info(
                f"Goal de ejecucion aceptado en fase '{self.current_motion_phase}'."
            )
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_execute_trajectory_result)
        except Exception as e:
            self.get_logger().error(
                f"Error recibiendo respuesta de ExecuteTrajectory: {e}"
            )
            self._finish_cycle()

    def _on_execute_trajectory_result(self, future):
        try:
            result_wrapper = future.result()
            if result_wrapper is None:
                self.get_logger().error("ExecuteTrajectory no devolvio resultado.")
                self._finish_cycle()
                return

            phase = self.current_motion_phase
            error_code = result_wrapper.result.error_code.val
            if error_code != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(
                    f"Fallo al ejecutar la trayectoria en fase '{phase}'. "
                    f"error_code={error_code}"
                )
                self._finish_cycle()
                return

            self.get_logger().info(
                f"Trayectoria ejecutada correctamente en fase '{phase}'."
            )
            self._advance_after_motion_success(phase)
        except Exception as e:
            self.get_logger().error(
                f"Error procesando resultado de ExecuteTrajectory: {e}"
            )
            self._finish_cycle()

    def _on_gripper_goal_response(self, future):
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.gripper_goal_active = False
                self.get_logger().error(
                    f"El gripper rechazo el goal en fase '{self.current_gripper_phase}'."
                )
                self._finish_cycle()
                return

            self.get_logger().info(
                f"Goal del gripper aceptado en fase '{self.current_gripper_phase}'."
            )
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_gripper_goal_result)
        except Exception as e:
            self.gripper_goal_active = False
            self.get_logger().error(f"Error enviando goal al gripper: {e}")
            self._finish_cycle()

    def _on_gripper_goal_result(self, future):
        try:
            result_wrapper = future.result()
            phase = self.current_gripper_phase
            self.gripper_goal_active = False

            if result_wrapper is None:
                self.get_logger().error("El gripper no devolvio resultado.")
                self._finish_cycle()
                return

            self.get_logger().info(
                f"Comando de gripper completado en fase '{phase}'."
            )

            if phase == "after_close":
                lift_target_pose = self._build_post_grasp_lift_pose()
                if lift_target_pose is None:
                    self._finish_cycle()
                    return

                if not self._send_cartesian_pose_goal(
                    lift_target_pose,
                    "post_grasp_lift",
                    "lift vertical post-grasp",
                ):
                    self._finish_cycle()
            elif phase == "after_open":
                self._send_joint_goal(
                    self._joint_state_from_positions(self.home_joint_positions),
                    "home",
                    "home",
                )
            elif phase == "verify_open_at_home":
                self.get_logger().info(
                    "Ciclo grasp->place->home completado con gripper verificado en abierto."
                )
                self._finish_cycle(
                    success=True,
                    message="Ciclo grasp->place->home completado con gripper abierto en home",
                )
            else:
                self._finish_cycle()
        except Exception as e:
            self.gripper_goal_active = False
            self.get_logger().error(f"Error procesando resultado del gripper: {e}")
            self._finish_cycle()

    def _on_best_grasp(self, msg: PoseStamped):
        self.latest_best_grasp_msg = msg
        self.latest_best_grasp_arrival_ns = self.get_clock().now().nanoseconds

        if not self.awaiting_grasp_result:
            return

        msg_stamp_ns = self._msg_stamp_to_ns(msg.header.stamp)
        if (
            msg_stamp_ns > 0
            and msg_stamp_ns < self.current_request_start_ns
            and self.latest_best_grasp_arrival_ns < self.current_request_start_ns
        ):
            return

        self.awaiting_grasp_result = False
        self._process_grasp_candidates(
            msg.header.frame_id,
            [msg.pose],
            source_label="best_grasp_fallback",
        )

    def _on_top_grasps(self, msg: PoseArray):
        self.latest_top_grasps_msg = msg
        self.latest_top_grasps_arrival_ns = self.get_clock().now().nanoseconds

        if not self.awaiting_grasp_result or len(msg.poses) == 0:
            return

        msg_stamp_ns = self._msg_stamp_to_ns(msg.header.stamp)
        if (
            msg_stamp_ns > 0
            and msg_stamp_ns < self.current_request_start_ns
            and self.latest_top_grasps_arrival_ns < self.current_request_start_ns
        ):
            return

        self.awaiting_grasp_result = False
        self._process_grasp_candidates(
            msg.header.frame_id,
            msg.poses,
            source_label="top_grasps",
        )

    def _process_grasp_candidates(
        self, frame_id: str, poses, source_label: str, retry_on_failure: bool = True
    ):
        try:
            if self.latest_arm_joint_state is None:
                self.get_logger().warn(
                    "Aun no hay /joint_states completos del brazo; se ignora el grasp."
                )
                if retry_on_failure:
                    self._retry_grasp("sin joint_states del brazo")
                else:
                    self._finish_cycle(
                        message="Sin /joint_states del brazo durante ejecucion cacheada"
                    )
                return

            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                frame_id,
                rclpy.time.Time(),
            )

            for idx, pose in enumerate(poses, start=1):
                target_pose = do_transform_pose(pose, transform)

                if not self._is_target_reachable(target_pose):
                    self.get_logger().info(
                        f"Candidato {idx}/{len(poses)} descartado por alcance "
                        f"(source='{source_label}')."
                    )
                    continue

                ik_pose = self._convert_pose_for_ik_link(target_pose)
                goal_state_info = self._solve_ik(ik_pose, self.latest_arm_joint_state)
                if goal_state_info is None:
                    self.get_logger().info(
                        f"Candidato {idx}/{len(poses)} sin IK valida "
                        f"(source='{source_label}')."
                    )
                    continue

                goal_joint_state, joint_distance = goal_state_info
                self.current_retry_count = 0
                self.pending_grasp_goal_state = goal_joint_state
                self.pending_grasp_target_pose = self._build_cartesian_grasp_pose(
                    target_pose
                )

                if self.use_approach_stage and self.approach_offset > 0.0:
                    approach_target_pose = self._offset_pose_along_local_axis(
                        target_pose,
                        self.approach_axis,
                        self.approach_sign * self.approach_offset,
                    )

                    if not self._is_target_reachable(approach_target_pose):
                        self.get_logger().info(
                            f"Candidato {idx}/{len(poses)} sin approach alcanzable "
                            f"(source='{source_label}')."
                        )
                        continue

                    approach_ik_pose = self._convert_pose_for_ik_link(approach_target_pose)
                    approach_goal_info = self._solve_ik(
                        approach_ik_pose, self.latest_arm_joint_state
                    )
                    if approach_goal_info is None:
                        self.get_logger().info(
                            f"Candidato {idx}/{len(poses)} con IK invalida para approach "
                            f"(source='{source_label}')."
                        )
                        continue

                    approach_joint_state, approach_joint_distance = approach_goal_info
                    self._send_joint_goal(
                        approach_joint_state,
                        "approach",
                        "approach/pregrasp",
                    )
                    self.get_logger().info(
                        f"Usando candidato {idx}/{len(poses)} de '{source_label}'. "
                        f"Approach resuelto con axis='{self.approach_axis}', "
                        f"offset={self.approach_sign * self.approach_offset:.3f} m, "
                        f"distancia_joint={approach_joint_distance:.3f} rad"
                    )
                else:
                    self._send_joint_goal(goal_joint_state, "grasp", "grasp final")
                    self.get_logger().info(
                        f"Usando candidato {idx}/{len(poses)} de '{source_label}' sin approach."
                    )

                q_start = ", ".join(
                    f"{name}={position:.3f}"
                    for name, position in zip(
                        self.latest_arm_joint_state.name,
                        self.latest_arm_joint_state.position,
                    )
                )
                q_goal = ", ".join(
                    f"{name}={position:.3f}"
                    for name, position in zip(
                        goal_joint_state.name,
                        goal_joint_state.position,
                    )
                )

                self.get_logger().info(
                    "Objetivo de grasp resuelto por IK. "
                    f"candidate={idx}/{len(poses)} | "
                    f"source='{source_label}' | "
                    f"distancia_joint={joint_distance:.3f} rad | "
                    f"q_start=[{q_start}] | q_goal=[{q_goal}]"
                )
                return

            self.get_logger().warn(
                f"Ninguno de los {len(poses)} grasps candidatos produjo una IK/approach valida "
                f"(source='{source_label}')."
            )
            if retry_on_failure:
                self._retry_grasp("sin candidatos con IK valida")
            else:
                self._finish_cycle(
                    message="Ningun grasp cacheado produjo una IK/approach valida"
                )

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            if retry_on_failure:
                self._retry_grasp("excepcion procesando grasp")
            else:
                self._finish_cycle(
                    message=f"Excepcion procesando grasps cacheados: {e}"
                )


def main():
    rclpy.init()
    node = MinimalGraspExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
