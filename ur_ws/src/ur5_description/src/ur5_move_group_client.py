#!/usr/bin/env python3

import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
    PlanningSceneWorld,
    RobotState,
)
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
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
        self.ik_link = self.declare_parameter("ik_link", "robotiq_hande_end").value
        self.grasp_pose_link = self.declare_parameter(
            "grasp_pose_link", self.ik_link
        ).value
        self.grasp_service_name = self.declare_parameter(
            "grasp_service_name", "/graspgen/run_inference"
        ).value
        self.run_cycle_service_name = self.declare_parameter(
            "run_cycle_service_name", "/ur5/run_grasp_cycle"
        ).value
        self.ik_service_name = self.declare_parameter(
            "ik_service_name", "/compute_ik"
        ).value
        self.grasp_result_timeout_sec = float(
            self.declare_parameter("grasp_result_timeout_sec", 5.0).value
        )
        self.max_ik_retries = int(
            self.declare_parameter("max_ik_retries", 5).value
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
                [1.7453, -1.8675, -0.7156, -1.7802, 1.7279, 2.6354],
            ).value
        )
        self.home_joint_positions = list(
            self.declare_parameter(
                "home_joint_positions",
                [0.8901, -1.0472, -2.0245, -1.0297, 1.4835, 3.3335],
            ).value
        )

        self.table_top_z = 0.0
        self.min_target_z = 0.03
        self.goal_joint_tolerance = 1e-3
        self.ik_timeout = Duration(sec=0, nanosec=250_000_000)
        self.latest_arm_joint_state = None
        self.latest_best_grasp_msg = None
        self.latest_best_grasp_arrival_ns = 0
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
        self.grasp_client = self.create_client(
            Trigger,
            self.grasp_service_name,
            callback_group=self.moveit_cb_group,
        )
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            self.gripper_action_name,
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

        # Subscribers
        self.create_subscription(
            PoseStamped,
            "/graspgen/best_grasp",
            self._on_best_grasp,
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
            "Nodo listo. Coordina grasp -> IK -> MoveIt "
            f"usando link objetivo '{self.ik_link}' "
            f"(grasp_pose_link='{self.grasp_pose_link}', execute={self.execute}, "
            f"auto_start={self.auto_start}, planning_pipeline='{self.planning_pipeline}', "
            f"planner_id='{self.planner_id}')."
        )

    def _publish_table(self):
        co = CollisionObject(id="table", operation=CollisionObject.ADD)
        co.header.frame_id = self.target_frame

        co.primitives.append(
            SolidPrimitive(
                type=SolidPrimitive.BOX,
                dimensions=[1.0, 1.0, 0.05],
            )
        )

        p = Pose()
        p.position.z = -0.0255
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
                    f"Timeout esperando /graspgen/best_grasp para request_id={self.current_request_id}."
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

    def _start_new_cycle(self, reason: str):
        if self.cycle_active:
            self.get_logger().warn("Ya hay un ciclo grasp->IK->MoveIt en ejecucion.")
            return False

        if self.latest_arm_joint_state is None:
            self.get_logger().warn(
                "No se puede iniciar el ciclo aun: faltan /joint_states del brazo."
            )
            return False

        self.cycle_active = True
        self.current_retry_count = 0
        self._request_new_grasp(reason)
        return True

    def _finish_cycle(self):
        self.cycle_active = False
        self.awaiting_grasp_result = False
        self.request_in_flight = False
        self.current_motion_phase = "idle"
        self.current_gripper_phase = "idle"
        self.gripper_goal_active = False

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
                f"Servicio de grasp completado. Esperando PoseStamped fresco "
                f"(request_id={self.current_request_id})."
            )

            # El grasp puede haber llegado antes de la respuesta del servicio.
            if (
                self.latest_best_grasp_msg is not None
                and self.latest_best_grasp_arrival_ns >= self.current_request_start_ns
                and self.awaiting_grasp_result
            ):
                self.awaiting_grasp_result = False
                self._process_grasp_pose(self.latest_best_grasp_msg)
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

    def _convert_pose_for_ik_link(self, target_pose: Pose) -> Pose:
        if self.grasp_pose_link == self.ik_link:
            return target_pose

        transform = self.tf_buffer.lookup_transform(
            self.grasp_pose_link,
            self.ik_link,
            rclpy.time.Time(),
        )
        return self._compose_pose_with_offset(target_pose, transform.transform)

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

    def _send_joint_goal(self, goal_joint_state: JointState, phase: str, log_label: str):
        if self.latest_arm_joint_state is None:
            self.get_logger().warn(
                f"No hay /joint_states completos del brazo; no puedo enviar goal para {log_label}."
            )
            self._finish_cycle()
            return

        goal = MoveGroup.Goal()
        goal.planning_options.plan_only = not self.execute

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

        self.current_motion_phase = phase
        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_move_group_response)

        q_goal = ", ".join(
            f"{name}={position:.3f}"
            for name, position in zip(goal_joint_state.name, goal_joint_state.position)
        )
        self.get_logger().info(
            f"{'Ejecucion' if self.execute else 'Plan'} solicitada para {log_label}. "
            f"pipeline='{self.planning_pipeline or 'default'}' | "
            f"planner_id='{self.planner_id or 'default'}' | "
            f"q_goal=[{q_goal}]"
        )

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

            mode = "ejecucion" if self.execute else "planificacion"
            self.get_logger().info(
                f"Goal aceptado para {mode} en fase '{self.current_motion_phase}'."
            )
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_move_group_result)
        except Exception as e:
            self.get_logger().error(f"Error recibiendo respuesta de MoveGroup: {e}")
            self._finish_cycle()

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
                if self.execute:
                    self.get_logger().info(
                        f"Trayectoria ejecutada correctamente en fase '{phase}'."
                    )
                else:
                    self.get_logger().info(
                        f"Plan generado correctamente en fase '{phase}'."
                    )
                    self._finish_cycle()
                    return

                if phase == "grasp":
                    self._send_gripper_goal(
                        self.close_gripper_position,
                        self.close_gripper_max_effort,
                        "after_close",
                        "cerrar grasp",
                    )
                elif phase == "place":
                    self._send_gripper_goal(
                        self.open_gripper_position,
                        self.open_gripper_max_effort,
                        "after_open",
                        "soltar objeto",
                    )
                elif phase == "home":
                    self.get_logger().info("Ciclo grasp->place->home completado.")
                    self._finish_cycle()
                else:
                    self._finish_cycle()
            else:
                action_name = "ejecutar" if self.execute else "planificar"
                self.get_logger().error(
                    f"Fallo al {action_name} la trayectoria en fase '{self.current_motion_phase}'. "
                    f"error_code={error_code}"
                )
                self._finish_cycle()
        except Exception as e:
            self.get_logger().error(f"Error procesando resultado de MoveGroup: {e}")
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
                self._send_joint_goal(
                    self._joint_state_from_positions(self.place_joint_positions),
                    "place",
                    "deposito/place",
                )
            elif phase == "after_open":
                self._send_joint_goal(
                    self._joint_state_from_positions(self.home_joint_positions),
                    "home",
                    "home",
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
        self._process_grasp_pose(msg)

    def _process_grasp_pose(self, msg: PoseStamped):
        try:
            if self.latest_arm_joint_state is None:
                self.get_logger().warn(
                    "Aun no hay /joint_states completos del brazo; se ignora el grasp."
                )
                self._retry_grasp("sin joint_states del brazo")
                return

            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
            )
            target_pose = do_transform_pose(msg.pose, transform)

            if not self._is_target_reachable(target_pose):
                self._retry_grasp("grasp fuera de alcance")
                return

            ik_pose = self._convert_pose_for_ik_link(target_pose)
            goal_state_info = self._solve_ik(ik_pose, self.latest_arm_joint_state)
            if goal_state_info is None:
                self._retry_grasp("IK invalida")
                return

            goal_joint_state, joint_distance = goal_state_info
            self.current_retry_count = 0
            self._send_joint_goal(goal_joint_state, "grasp", "grasp")

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
                f"distancia_joint={joint_distance:.3f} rad | "
                f"q_start=[{q_start}] | q_goal=[{q_goal}]"
            )

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            self._retry_grasp("excepcion procesando grasp")


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
