#!/usr/bin/env python3

import time
from enum import Enum, auto
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from std_srvs.srv import Empty


class BTStatus(Enum):
    SUCCESS = auto()
    FAILURE = auto()
    RUNNING = auto()

class BTNode:
    def __init__(self, name: str):
        self.name = name

    def tick(self):
        raise NotImplementedError

    def reset(self):
        pass

class SequenceNode(BTNode):
    def __init__(self, name: str, children):
        super().__init__(name)
        self.children = children
        self.current_index = 0

    def tick(self):
        while self.current_index < len(self.children):
            status = self.children[self.current_index].tick()
            if status == BTStatus.SUCCESS:
                self.current_index += 1
                continue
            return status
        return BTStatus.SUCCESS

    def reset(self):
        self.current_index = 0
        for child in self.children:
            child.reset()


class RetryNode(BTNode):
    def __init__(self, name: str, child: BTNode, max_attempts: int, logger):
        super().__init__(name)
        self.child = child
        self.max_attempts = max(1, max_attempts)
        self.logger = logger
        self.attempts = 0

    def tick(self):
        status = self.child.tick()
        if status == BTStatus.SUCCESS:
            return BTStatus.SUCCESS
        if status == BTStatus.RUNNING:
            return BTStatus.RUNNING

        self.attempts += 1
        if self.attempts >= self.max_attempts:
            self.logger.error(
                f"BT agoto los reintentos ({self.max_attempts}) en '{self.child.name}'."
            )
            return BTStatus.FAILURE

        self.logger.warn(
            f"BT reintentando '{self.child.name}' "
            f"(intento {self.attempts + 1}/{self.max_attempts})."
        )
        self.child.reset()
        return BTStatus.RUNNING

    def reset(self):
        self.attempts = 0
        self.child.reset()


class ConditionNode(BTNode):
    def __init__(self, name: str, condition_fn):
        super().__init__(name)
        self.condition_fn = condition_fn

    def tick(self):
        return BTStatus.SUCCESS if self.condition_fn() else BTStatus.FAILURE


class OneShotActionNode(BTNode):
    def __init__(self, name: str, action_fn):
        super().__init__(name)
        self.action_fn = action_fn
        self.done = False
        self.result = BTStatus.FAILURE

    def tick(self):
        if not self.done:
            ok = self.action_fn()
            self.result = BTStatus.SUCCESS if ok else BTStatus.FAILURE
            self.done = True
        return self.result

    def reset(self):
        self.done = False
        self.result = BTStatus.FAILURE


class WaitForGraspsNode(BTNode):
    def __init__(self, name: str, ready_fn, timeout_fn):
        super().__init__(name)
        self.ready_fn = ready_fn
        self.timeout_fn = timeout_fn
        self.start_time = None

    def tick(self):
        if self.start_time is None:
            self.start_time = time.monotonic()

        if self.ready_fn():
            return BTStatus.SUCCESS

        if time.monotonic() - self.start_time > self.timeout_fn():
            return BTStatus.FAILURE

        return BTStatus.RUNNING

    def reset(self):
        self.start_time = None


class BTGraspCoordinator(Node):
    def __init__(self):
        super().__init__("ur5_bt_grasp_coordinator")

        self.cb_group = ReentrantCallbackGroup()

        self.sam_select_service_name = self.declare_parameter(
            "sam_select_service_name", "/sam2/select_object"
        ).value
        self.grasp_service_name = self.declare_parameter(
            "grasp_service_name", "/graspgen/run_inference"
        ).value
        self.sam_pause_service_name = self.declare_parameter(
            "sam_pause_service_name", "/sam2/pause_inference"
        ).value
        self.clear_octomap_service_name = self.declare_parameter(
            "clear_octomap_service_name", "/clear_octomap"
        ).value
        self.octomap_home_cloud_service_name = self.declare_parameter(
            "octomap_home_cloud_service_name",
            "/octomap_home_cloud_manager/capture_home_cloud",
        ).value
        self.execute_cached_cycle_service_name = self.declare_parameter(
            "execute_cached_cycle_service_name", "/ur5/execute_cached_grasp_cycle"
        ).value
        self.go_home_service_name = self.declare_parameter(
            "go_home_service_name", "/ur5/go_home"
        ).value
        self.prepare_bt_service_name = self.declare_parameter(
            "prepare_bt_service_name", "/ur5/prepare_bt_grasp_cycle"
        ).value
        self.execute_bt_service_name = self.declare_parameter(
            "execute_bt_service_name", "/ur5/execute_bt_grasp_cycle"
        ).value
        self.bt_service_name = self.declare_parameter(
            "bt_service_name", "/ur5/run_bt_grasp_cycle"
        ).value
        self.top_grasps_topic = self.declare_parameter(
            "top_grasps_topic", "/graspgen/top_grasps"
        ).value
        self.max_retries = int(self.declare_parameter("max_retries", 3).value)
        self.grasp_wait_timeout_sec = float(
            self.declare_parameter("grasp_wait_timeout_sec", 20.0).value
        )
        self.execution_timeout_sec = float(
            self.declare_parameter("execution_timeout_sec", 120.0).value
        )
        self.tick_period_sec = float(
            self.declare_parameter("tick_period_sec", 0.1).value
        )
        self.home_joint_tolerance = float(
            self.declare_parameter("home_joint_tolerance", 0.09).value
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
        self.home_joint_positions = list(
            self.declare_parameter(
                "home_joint_positions",
                [0.8901, -1.0472, -2.0245, -1.0297, 1.4835, 0.0],
            ).value
        )

        self.latest_arm_joint_state = None
        self.latest_best_grasp_msg = None
        self.latest_best_grasp_arrival_ns = 0
        self.latest_top_grasps_msg = None
        self.latest_top_grasps_arrival_ns = 0
        self.current_request_start_ns = 0
        self.workflow_active = False
        self.planning_ready = False
        self.planning_summary = "Sin planificacion preparada"
        self.last_failure_reason = "Sin fallo registrado"
        self.selection_done_for_cycle = False

        self.sam_client = self.create_client(
            Trigger,
            self.sam_select_service_name,
            callback_group=self.cb_group,
        )
        self.sam_pause_client = self.create_client(
            Trigger,
            self.sam_pause_service_name,
            callback_group=self.cb_group,
        )
        self.clear_octomap_client = self.create_client(
            Empty,
            self.clear_octomap_service_name,
            callback_group=self.cb_group,
        )
        self.octomap_home_cloud_client = self.create_client(
            Trigger,
            self.octomap_home_cloud_service_name,
            callback_group=self.cb_group,
        )
        self.grasp_client = self.create_client(
            Trigger,
            self.grasp_service_name,
            callback_group=self.cb_group,
        )
        self.execute_cached_cycle_client = self.create_client(
            Trigger,
            self.execute_cached_cycle_service_name,
            callback_group=self.cb_group,
        )
        self.go_home_client = self.create_client(
            Trigger,
            self.go_home_service_name,
            callback_group=self.cb_group,
        )

        self.create_subscription(
            PoseStamped,
            "/graspgen/best_grasp",
            self._on_best_grasp,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            PoseArray,
            self.top_grasps_topic,
            self._on_top_grasps,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            50,
            callback_group=self.cb_group,
        )

        self.prepare_bt_service = self.create_service(
            Trigger,
            self.prepare_bt_service_name,
            self._handle_prepare_bt_cycle,
            callback_group=self.cb_group,
        )
        self.execute_bt_service = self.create_service(
            Trigger,
            self.execute_bt_service_name,
            self._handle_execute_bt_cycle,
            callback_group=self.cb_group,
        )
        self.run_bt_service = self.create_service(
            Trigger,
            self.bt_service_name,
            self._handle_run_bt_cycle,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            "BT coordinator listo "
            f"(prepare='{self.prepare_bt_service_name}', execute='{self.execute_bt_service_name}', "
            f"legacy='{self.bt_service_name}', sam='{self.sam_select_service_name}', "
            f"grasp='{self.grasp_service_name}', execute_cached='{self.execute_cached_cycle_service_name}')."
        )

    def _wait_for_future_result(self, future, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)

        if not future.done():
            return None
        return future.result()

    def _msg_stamp_to_ns(self, stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _on_joint_states(self, msg: JointState):
        joint_map = {name: position for name, position in zip(msg.name, msg.position)}
        if not all(name in joint_map for name in self.arm_joint_names):
            return

        arm_state = JointState()
        arm_state.header = msg.header
        arm_state.name = list(self.arm_joint_names)
        arm_state.position = [joint_map[name] for name in self.arm_joint_names]
        self.latest_arm_joint_state = arm_state

    def _on_best_grasp(self, msg: PoseStamped):
        self.latest_best_grasp_msg = msg
        self.latest_best_grasp_arrival_ns = self.get_clock().now().nanoseconds

    def _on_top_grasps(self, msg: PoseArray):
        self.latest_top_grasps_msg = msg
        self.latest_top_grasps_arrival_ns = self.get_clock().now().nanoseconds

    def _has_valid_joint_state(self) -> bool:
        if self.latest_arm_joint_state is None:
            self.last_failure_reason = "Aun no hay /joint_states completos del brazo"
            self.get_logger().warn("BT: aun no hay /joint_states completos del brazo.")
            return False
        return True

    def _is_in_home(self) -> bool:
        if self.latest_arm_joint_state is None:
            self.last_failure_reason = "No puedo validar home sin /joint_states"
            self.get_logger().warn("BT: no puedo validar home sin /joint_states.")
            return False

        if len(self.latest_arm_joint_state.position) != len(self.home_joint_positions):
            self.last_failure_reason = (
                "home_joint_positions no coincide con arm_joint_names"
            )
            self.get_logger().error(
                "BT: home_joint_positions no coincide con el largo de arm_joint_names."
            )
            return False

        deltas = [
            abs(current - home)
            for current, home in zip(
                self.latest_arm_joint_state.position, self.home_joint_positions
            )
        ]
        max_delta = max(deltas) if deltas else float("inf")
        if max_delta > self.home_joint_tolerance:
            self.last_failure_reason = (
                f"El brazo no esta en home (max_delta={max_delta:.3f} rad)"
            )
            self.get_logger().warn(
                f"BT: el brazo no esta en home (max_delta={max_delta:.3f} rad, "
                f"tol={self.home_joint_tolerance:.3f})."
            )
            return False
        return True

    def _call_trigger(self, client, service_name: str, timeout_sec: float, label: str):
        if not client.wait_for_service(timeout_sec=1.0):
            self.last_failure_reason = f"Servicio '{service_name}' no disponible"
            self.get_logger().error(
                f"BT: servicio '{service_name}' no disponible para {label}."
            )
            return False, f"Servicio '{service_name}' no disponible"

        future = client.call_async(Trigger.Request())
        response = self._wait_for_future_result(future, timeout_sec=timeout_sec)
        if response is None:
            self.last_failure_reason = f"Timeout en '{service_name}' durante {label}"
            self.get_logger().error(
                f"BT: timeout llamando a '{service_name}' para {label}."
            )
            return False, f"Timeout en '{service_name}'"

        if not response.success:
            self.last_failure_reason = (
                f"Fallo en '{service_name}' durante {label}: {response.message}"
            )
            self.get_logger().warn(
                f"BT: '{service_name}' fallo durante {label}: {response.message}"
            )
            return False, response.message

        self.get_logger().info(
            f"BT: '{service_name}' completado para {label}: {response.message}"
        )
        return True, response.message

    def _call_sam_selection(self) -> bool:
        if self.selection_done_for_cycle:
            self.get_logger().info(
                "BT: la seleccion SAM ya fue realizada en este ciclo de preparacion."
            )
            return True

        ok, _ = self._call_trigger(
            self.sam_client,
            self.sam_select_service_name,
            timeout_sec=120.0,
            label="seleccion SAM",
        )
        if ok:
            self.selection_done_for_cycle = True
        return ok

    def _pause_sam(self) -> bool:
        ok, _ = self._call_trigger(
            self.sam_pause_client,
            self.sam_pause_service_name,
            timeout_sec=5.0,
            label="pausa SAM previa a ejecucion",
        )
        return ok

    def _clear_octomap(self) -> bool:
        if not self.clear_octomap_client.wait_for_service(timeout_sec=1.0):
            self.last_failure_reason = (
                f"Servicio '{self.clear_octomap_service_name}' no disponible"
            )
            self.get_logger().error(
                f"BT: servicio '{self.clear_octomap_service_name}' no disponible para limpiar OctoMap."
            )
            return False

        future = self.clear_octomap_client.call_async(Empty.Request())
        response = self._wait_for_future_result(future, timeout_sec=2.0)
        if response is None:
            self.last_failure_reason = (
                f"Timeout en '{self.clear_octomap_service_name}' durante limpieza de OctoMap"
            )
            self.get_logger().error(
                f"BT: timeout llamando a '{self.clear_octomap_service_name}' para limpiar OctoMap."
            )
            return False

        self.get_logger().info(
            f"BT: '{self.clear_octomap_service_name}' completado para limpieza de OctoMap."
        )
        return True

    def _capture_octomap_home_cloud(self) -> bool:
        ok, _ = self._call_trigger(
            self.octomap_home_cloud_client,
            self.octomap_home_cloud_service_name,
            timeout_sec=8.0,
            label="captura de nube home para OctoMap",
        )
        return ok

    def _ensure_home(self) -> bool:
        if self._is_in_home():
            self.get_logger().info("BT: el brazo ya esta en home al iniciar.")
            return True

        self.get_logger().info("BT: el brazo no esta en home, solicitando retorno a home.")
        ok, _ = self._call_trigger(
            self.go_home_client,
            self.go_home_service_name,
            timeout_sec=90.0,
            label="retorno a home previo a seleccion",
        )
        return ok

    def _call_grasp_generation(self) -> bool:
        self.current_request_start_ns = self.get_clock().now().nanoseconds
        ok, _ = self._call_trigger(
            self.grasp_client,
            self.grasp_service_name,
            timeout_sec=self.grasp_wait_timeout_sec,
            label="inferencia GraspGen",
        )
        return ok

    def _has_fresh_grasps(self) -> bool:
        if (
            self.latest_top_grasps_msg is not None
            and len(self.latest_top_grasps_msg.poses) > 0
        ):
            msg_stamp_ns = self._msg_stamp_to_ns(self.latest_top_grasps_msg.header.stamp)
            if (
                self.latest_top_grasps_arrival_ns >= self.current_request_start_ns
                or msg_stamp_ns >= self.current_request_start_ns
            ):
                return True

        if self.latest_best_grasp_msg is not None:
            msg_stamp_ns = self._msg_stamp_to_ns(self.latest_best_grasp_msg.header.stamp)
            if (
                self.latest_best_grasp_arrival_ns >= self.current_request_start_ns
                or msg_stamp_ns >= self.current_request_start_ns
            ):
                return True

        self.last_failure_reason = "No llegaron grasps frescos desde GraspGen"
        return False

    def _execute_cached_cycle(self) -> bool:
        ok, _ = self._call_trigger(
            self.execute_cached_cycle_client,
            self.execute_cached_cycle_service_name,
            timeout_sec=self.execution_timeout_sec,
            label="ejecucion cacheada MoveGroup",
        )
        return ok

    def _build_prepare_tree(self) -> BTNode:
        planning_attempt_sequence = SequenceNode(
            "GraspPlanningSequence",
            [
                OneShotActionNode("CallGraspGen", self._call_grasp_generation),
                WaitForGraspsNode(
                    "WaitForFreshGrasps",
                    self._has_fresh_grasps,
                    lambda: self.grasp_wait_timeout_sec,
                )
            ],
        )
        return SequenceNode(
            "RootPrepareTree",
            [
                ConditionNode("JointStatesReady", self._has_valid_joint_state),
                OneShotActionNode("EnsureHome", self._ensure_home),
                ConditionNode("RobotAtHome", self._is_in_home),
                OneShotActionNode("CallSAM", self._call_sam_selection),
                OneShotActionNode("OctoHomeCloud", self._capture_octomap_home_cloud),
                OneShotActionNode("ClearOctomap", self._clear_octomap),
                RetryNode(
                    "RetryPlanning",
                    planning_attempt_sequence,
                    max_attempts=self.max_retries,
                    logger=self.get_logger(),
                ),
                
            ],
        )

    def _run_tree_until_done(self, root: BTNode, start_label: str):
        self.get_logger().info(start_label)
        while rclpy.ok():
            status = root.tick()
            if status == BTStatus.SUCCESS:
                return True
            if status == BTStatus.FAILURE:
                return False
            time.sleep(self.tick_period_sec)
        return False

    def _handle_prepare_bt_cycle(self, request, response):
        del request

        if self.workflow_active:
            response.success = False
            response.message = "Ya hay una accion BT en ejecucion"
            return response

        self.workflow_active = True
        root = self._build_prepare_tree()
        try:
            self.selection_done_for_cycle = False
            ok = self._run_tree_until_done(root, "BT: iniciando preparacion desde home.")
            if ok:
                self.planning_ready = True
                self.planning_summary = (
                    "Preparacion completada: objeto seleccionado y grasps frescos disponibles"
                )
                response.success = True
                response.message = self.planning_summary
            else:
                self.planning_ready = False
                self.planning_summary = (
                    f"BT fallo durante la preparacion: {self.last_failure_reason}"
                )
                response.success = False
                response.message = self.planning_summary
            return response
        finally:
            root.reset()
            self.workflow_active = False

    def _handle_execute_bt_cycle(self, request, response):
        del request

        if self.workflow_active:
            response.success = False
            response.message = "Ya hay una accion BT en ejecucion"
            return response

        if not self.planning_ready:
            response.success = False
            response.message = (
                "No hay una preparacion lista. Primero llama a prepare_bt_grasp_cycle."
            )
            return response

        self.workflow_active = True
        try:
            if not self._pause_sam():
                response.success = False
                response.message = "No se pudo pausar SAM antes de ejecutar"
                return response

            ok = self._execute_cached_cycle()
            self.planning_ready = False
            if ok:
                self.planning_summary = "Ejecucion BT completada"
                response.success = True
                response.message = self.planning_summary
            else:
                self.planning_summary = "Ejecucion BT fallida"
                response.success = False
                response.message = self.planning_summary
            return response
        finally:
            self.selection_done_for_cycle = False
            self.workflow_active = False

    def _handle_run_bt_cycle(self, request, response):
        del request

        if self.workflow_active:
            response.success = False
            response.message = "Ya hay un arbol de comportamiento en ejecucion"
            return response

        prepare_response = Trigger.Response()
        prepare_response = self._handle_prepare_bt_cycle(
            Trigger.Request(), prepare_response
        )
        if not prepare_response.success:
            response.success = False
            response.message = prepare_response.message
            return response

        execute_response = Trigger.Response()
        execute_response = self._handle_execute_bt_cycle(
            Trigger.Request(), execute_response
        )
        response.success = execute_response.success
        response.message = execute_response.message
        return response


def main():
    rclpy.init()
    node = BTGraspCoordinator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
