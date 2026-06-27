#!/usr/bin/env python3

import json
import subprocess
import time
import urllib.error
import urllib.request
from enum import Enum, auto
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from std_srvs.srv import Empty


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


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
        self.grasp_sam_service_name = self.declare_parameter(
            "grasp_sam_service_name", "/graspgen/run_inference_sam"
        ).value
        self.sam_release_service_name = self.declare_parameter(
            "sam_release_service_name", "/sam2/release_model"
        ).value
        self.sam_load_service_name = self.declare_parameter(
            "sam_load_service_name", "/sam2/load_model"
        ).value
        self.sam_apply_vlm_prompt_service_name = self.declare_parameter(
            "sam_apply_vlm_prompt_service_name", "/sam2/apply_vlm_prompt"
        ).value
        self.graspgen_release_service_name = self.declare_parameter(
            "graspgen_release_service_name", "/graspgen/release_model"
        ).value
        self.graspgen_load_service_name = self.declare_parameter(
            "graspgen_load_service_name", "/graspgen/load_model"
        ).value
        self.vlm_model_swap_wait_sec = float(
            self.declare_parameter("vlm_model_swap_wait_sec", 1.5).value
        )
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
        self.execute_pick_service_name = self.declare_parameter(
            "execute_pick_service_name", "/ur5/execute_pick"
        ).value
        self.execute_place_service_name = self.declare_parameter(
            "execute_place_service_name", "/ur5/execute_place"
        ).value
        self.clear_grasp_cache_service_name = self.declare_parameter(
            "clear_grasp_cache_service_name", "/ur5/clear_grasp_cache"
        ).value
        self.go_home_service_name = self.declare_parameter(
            "go_home_service_name", "/ur5/go_home"
        ).value
        self.prepare_bt_service_name = self.declare_parameter(
            "prepare_bt_service_name", "/ur5/prepare_bt_grasp_cycle"
        ).value
        self.prepare_context_bt_service_name = self.declare_parameter(
            "prepare_context_bt_service_name", "/ur5/prepare_context_bt_grasp_cycle"
        ).value
        self.execute_bt_service_name = self.declare_parameter(
            "execute_bt_service_name", "/ur5/execute_bt_grasp_cycle"
        ).value
        self.bt_service_name = self.declare_parameter(
            "bt_service_name", "/ur5/run_bt_grasp_cycle"
        ).value
        self.clean_table_service_name = self.declare_parameter(
            "clean_table_service_name", "/ur5/clean_table"
        ).value
        self.clean_table_max_cycles = int(
            self.declare_parameter("clean_table_max_cycles", 20).value
        )
        self.clean_table_use_vlm = _as_bool(
            self.declare_parameter("clean_table_use_vlm", False).value
        )
        self.clean_table_scan_on_no_grasps = _as_bool(
            self.declare_parameter("clean_table_scan_on_no_grasps", True).value
        )
        self.clean_table_treat_selection_failure_as_done = _as_bool(
            self.declare_parameter(
                "clean_table_treat_selection_failure_as_done", True
            ).value
        )
        self.scan_prepare_bt_service_name = self.declare_parameter(
            "scan_prepare_bt_service_name", "/ur5/prepare_scan_bt_grasp_cycle"
        ).value
        self.capture_scan_service_name = self.declare_parameter(
            "capture_scan_service_name", "/point_cloud_scanner/capture_scan"
        ).value
        self.vlm_health_url = self.declare_parameter(
            "vlm_health_url", "http://192.168.1.110:8888/health"
        ).value
        self.vlm_release_url = self.declare_parameter(
            "vlm_release_url", "http://192.168.1.110:8888/release"
        ).value
        self.vlm_select_service_name = self.declare_parameter(
            "vlm_select_service_name", "/vlm_prompt/select_object"
        ).value
        self.vlm_select_next_service_name = self.declare_parameter(
            "vlm_select_next_service_name", "/vlm_prompt/select_next_object"
        ).value
        self.vlm_reset_handled_service_name = self.declare_parameter(
            "vlm_reset_handled_service_name", "/vlm_prompt/reset_handled_objects"
        ).value
        self.vlm_health_timeout_sec = float(
            self.declare_parameter("vlm_health_timeout_sec", 60.0).value
        )
        self.vlm_release_timeout_sec = float(
            self.declare_parameter("vlm_release_timeout_sec", 20.0).value
        )
        # Tras /release, esperar a que Gemma confirme por /health que el modelo
        # quedo descargado y su VRAM por debajo del umbral, antes de recargar
        # SAM2. Evita un OOM cuando Gemma sigue residente en GPU.
        self.vlm_release_settle_timeout_sec = float(
            self.declare_parameter("vlm_release_settle_timeout_sec", 30.0).value
        )
        self.vlm_release_free_gb_threshold = float(
            self.declare_parameter("vlm_release_free_gb_threshold", 1.5).value
        )
        # Minimum GPU free memory (from nvidia-smi, host view) required before
        # reloading SAM2. This catches CUDA context memory that the Docker
        # container holds even after PyTorch reports cuda_allocated_gb ≈ 0.
        self.vlm_min_free_vram_gb = float(
            self.declare_parameter("vlm_min_free_vram_gb", 4.0).value
        )
        self.prepare_vlm_bt_service_name = self.declare_parameter(
            "prepare_vlm_bt_service_name", "/ur5/prepare_vlm_bt_grasp_cycle"
        ).value
        self.scan_prepare_vlm_bt_service_name = self.declare_parameter(
            "scan_prepare_vlm_bt_service_name", "/ur5/prepare_scan_vlm_bt_grasp_cycle"
        ).value
        self.top_grasps_topic = self.declare_parameter(
            "top_grasps_topic", "/graspgen/top_grasps"
        ).value
        self.max_retries = int(self.declare_parameter("max_retries", 3).value)
        self.sam_selection_retries = int(
            self.declare_parameter("sam_selection_retries", 3).value
        )
        self.grasp_wait_timeout_sec = float(
            self.declare_parameter("grasp_wait_timeout_sec", 20.0).value
        )
        self.service_wait_timeout_sec = float(
            self.declare_parameter("service_wait_timeout_sec", 10.0).value
        )
        self.scan_wait_timeout_sec = float(
            self.declare_parameter("scan_wait_timeout_sec", 180.0).value
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
        self.last_failure_type = ""
        self.last_failure_stage = ""
        self.last_cached_execution_message = "Sin ejecucion cacheada"
        self.selection_done_for_cycle = False

        self._failure_published_this_cycle = False

        # Publicador latcheado del fallo estructurado para el orquestador LangGraph.
        # El BT es duenio de las etapas de preparacion: segmentacion (SAM2) y
        # generacion de grasps (GraspGen). robot_skill_node lo reenvia como
        # failure_type/stage del contrato.
        skill_failure_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.skill_failure_pub = self.create_publisher(
            String, "/ur5/skill_failure", skill_failure_qos
        )

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
        self.grasp_sam_client = self.create_client(
            Trigger,
            self.grasp_sam_service_name,
            callback_group=self.cb_group,
        )
        self.execute_cached_cycle_client = self.create_client(
            Trigger,
            self.execute_cached_cycle_service_name,
            callback_group=self.cb_group,
        )
        self.execute_pick_client = self.create_client(
            Trigger,
            self.execute_pick_service_name,
            callback_group=self.cb_group,
        )
        self.execute_place_client = self.create_client(
            Trigger,
            self.execute_place_service_name,
            callback_group=self.cb_group,
        )
        self.clear_grasp_cache_client = self.create_client(
            Trigger,
            self.clear_grasp_cache_service_name,
            callback_group=self.cb_group,
        )
        self.go_home_client = self.create_client(
            Trigger,
            self.go_home_service_name,
            callback_group=self.cb_group,
        )
        self.capture_scan_client = self.create_client(
            Trigger,
            self.capture_scan_service_name,
            callback_group=self.cb_group,
        )
        self.vlm_select_client = self.create_client(
            Trigger,
            self.vlm_select_service_name,
            callback_group=self.cb_group,
        )
        self.vlm_select_next_client = self.create_client(
            Trigger,
            self.vlm_select_next_service_name,
            callback_group=self.cb_group,
        )
        self.vlm_reset_handled_client = self.create_client(
            Trigger,
            self.vlm_reset_handled_service_name,
            callback_group=self.cb_group,
        )
        self.sam_release_client = self.create_client(
            Trigger,
            self.sam_release_service_name,
            callback_group=self.cb_group,
        )
        self.sam_load_client = self.create_client(
            Trigger,
            self.sam_load_service_name,
            callback_group=self.cb_group,
        )
        self.sam_apply_vlm_prompt_client = self.create_client(
            Trigger,
            self.sam_apply_vlm_prompt_service_name,
            callback_group=self.cb_group,
        )
        self.graspgen_release_client = self.create_client(
            Trigger,
            self.graspgen_release_service_name,
            callback_group=self.cb_group,
        )
        self.graspgen_load_client = self.create_client(
            Trigger,
            self.graspgen_load_service_name,
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
        self.prepare_context_bt_service = self.create_service(
            Trigger,
            self.prepare_context_bt_service_name,
            self._handle_context_prepare_bt_cycle,
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
        self.clean_table_service = self.create_service(
            Trigger,
            self.clean_table_service_name,
            self._handle_clean_table,
            callback_group=self.cb_group,
        )
        self.scan_prepare_bt_service = self.create_service(
            Trigger,
            self.scan_prepare_bt_service_name,
            self._handle_scan_prepare_bt_cycle,
            callback_group=self.cb_group,
        )
        self.prepare_vlm_bt_service = self.create_service(
            Trigger,
            self.prepare_vlm_bt_service_name,
            self._handle_vlm_prepare_bt_cycle,
            callback_group=self.cb_group,
        )
        self.scan_prepare_vlm_bt_service = self.create_service(
            Trigger,
            self.scan_prepare_vlm_bt_service_name,
            self._handle_scan_vlm_prepare_bt_cycle,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            "BT coordinator listo "
            f"(prepare='{self.prepare_bt_service_name}', "
            f"prepare_context='{self.prepare_context_bt_service_name}', "
            f"prepare_scan='{self.scan_prepare_bt_service_name}', "
            f"prepare_vlm='{self.prepare_vlm_bt_service_name}', "
            f"prepare_scan_vlm='{self.scan_prepare_vlm_bt_service_name}', "
            f"execute='{self.execute_bt_service_name}', "
            f"legacy='{self.bt_service_name}', sam='{self.sam_select_service_name}', "
            f"vlm_select='{self.vlm_select_service_name}', "
            f"vlm_health='{self.vlm_health_url}', "
            f"vlm_release='{self.vlm_release_url}', "
            f"sam_release='{self.sam_release_service_name}', "
            f"sam_load='{self.sam_load_service_name}', "
            f"sam_apply='{self.sam_apply_vlm_prompt_service_name}', "
            f"graspgen_release='{self.graspgen_release_service_name}', "
            f"graspgen_load='{self.graspgen_load_service_name}', "
            f"grasp_sam='{self.grasp_sam_service_name}', "
            f"grasp_scan='{self.grasp_service_name}', "
            f"capture_scan='{self.capture_scan_service_name}', "
            f"execute_cached='{self.execute_cached_cycle_service_name}', "
            f"execute_pick='{self.execute_pick_service_name}', "
            f"execute_place='{self.execute_place_service_name}', "
            f"clean_table='{self.clean_table_service_name}')."
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

    def _publish_skill_failure(self, failure_type: str, stage: str, detail: str):
        """Publica (latcheado) el fallo estructurado para el orquestador."""
        payload = {
            "ok": not failure_type,
            "failure_type": failure_type,
            "stage": stage,
            "detail": detail,
            "stamp": time.time(),
            "source": "behavior_tree_node",
        }
        self.skill_failure_pub.publish(String(data=json.dumps(payload)))

    def _set_failure(self, failure_type: str, stage: str, reason: str):
        self.last_failure_reason = reason
        self.last_failure_type = failure_type
        self.last_failure_stage = stage
        self._failure_published_this_cycle = True
        self._publish_skill_failure(failure_type, stage, reason)

    def _clear_failure(self):
        """Marca inicio de un ciclo limpio (sin fallo) y lo publica."""
        self._failure_published_this_cycle = False
        self.last_failure_type = ""
        self.last_failure_stage = ""
        self._publish_skill_failure("", "", "preparacion en curso")

    def _call_trigger(self, client, service_name: str, timeout_sec: float, label: str):
        if not client.wait_for_service(timeout_sec=self.service_wait_timeout_sec):
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
        else:
            self._set_failure(
                "segmentation_failed", "segmentation", self.last_failure_reason
            )
        return ok

    def _clear_grasp_cache_for_selection(self) -> bool:
        self.latest_best_grasp_msg = None
        self.latest_best_grasp_arrival_ns = 0
        self.latest_top_grasps_msg = None
        self.latest_top_grasps_arrival_ns = 0
        self.current_request_start_ns = self.get_clock().now().nanoseconds
        self.planning_ready = False
        ok, _ = self._call_trigger(
            self.clear_grasp_cache_client,
            self.clear_grasp_cache_service_name,
            timeout_sec=5.0,
            label="invalidacion de grasps anteriores",
        )
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
        # GraspGen pudo haberse descargado de la GPU para el ciclo VLM; lo
        # recargamos antes de inferir. Es idempotente si ya estaba cargado.
        if not self._ensure_graspgen_loaded():
            return False
        self.current_request_start_ns = self.get_clock().now().nanoseconds
        ok, _ = self._call_trigger(
            self.grasp_sam_client,
            self.grasp_sam_service_name,
            timeout_sec=self.grasp_wait_timeout_sec,
            label="inferencia GraspGen (nube SAM2)",
        )
        if not ok:
            self._set_failure(
                "no_grasps_generated", "grasp_generation", self.last_failure_reason
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

        self._set_failure(
            "no_grasps_generated",
            "grasp_generation",
            "No llegaron grasps frescos desde GraspGen",
        )
        return False

    def _execute_cached_cycle(self) -> bool:
        ok, message = self._call_trigger(
            self.execute_cached_cycle_client,
            self.execute_cached_cycle_service_name,
            timeout_sec=self.execution_timeout_sec,
            label="ejecucion cacheada MoveGroup",
        )
        self.last_cached_execution_message = message
        return ok

    def _call_capture_scan(self) -> bool:
        """Inicia el scanner de nube de puntos y marca el instante de inicio para
        poder verificar si los grasps resultantes son frescos respecto a este scan."""
        self.current_request_start_ns = self.get_clock().now().nanoseconds
        ok, _ = self._call_trigger(
            self.capture_scan_client,
            self.capture_scan_service_name,
            timeout_sec=10.0,
            label="inicio de scanner de nube",
        )
        return ok

    def _build_sam_selection_retry_node(self) -> BTNode:
        return RetryNode(
            "RetrySAMSelection",
            OneShotActionNode("CallSAM", self._call_sam_selection),
            max_attempts=self.sam_selection_retries,
            logger=self.get_logger(),
        )

    def _build_scan_prepare_tree(self) -> BTNode:
        """Arbol de preparacion con scanner 3D.

        A diferencia de _build_prepare_tree (que llama a graspgen directamente),
        este arbol lanza el scanner de nube de puntos. El scanner ejecuta la
        trayectoria en semicirculo, acumula la nube del objeto y al terminar
        llama automaticamente a /graspgen/run_inference (auto_infer_after_scan).
        WaitForScanInference espera que ese resultado llegue.
        """
        return SequenceNode(
            "RootScanPrepareTree",
            [
                ConditionNode("JointStatesReady", self._has_valid_joint_state),
                OneShotActionNode("EnsureHome", self._ensure_home),
                ConditionNode("RobotAtHome", self._is_in_home),
                OneShotActionNode("ClearPreviousGrasps", self._clear_grasp_cache_for_selection),
                self._build_sam_selection_retry_node(),
                OneShotActionNode("OctoHomeCloud", self._capture_octomap_home_cloud),
                OneShotActionNode("ClearOctomap", self._clear_octomap),
                OneShotActionNode("CallCaptureScanner", self._call_capture_scan),
                WaitForGraspsNode(
                    "WaitForScanInference",
                    self._has_fresh_grasps,
                    lambda: self.scan_wait_timeout_sec,
                ),
            ],
        )

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
                OneShotActionNode("ClearPreviousGrasps", self._clear_grasp_cache_for_selection),
                self._build_sam_selection_retry_node(),
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

    def _build_context_prepare_tree(self) -> BTNode:
        planning_attempt_sequence = SequenceNode(
            "ContextGraspPlanningSequence",
            [
                OneShotActionNode("CallGraspGen", self._call_grasp_generation),
                WaitForGraspsNode(
                    "WaitForFreshGrasps",
                    self._has_fresh_grasps,
                    lambda: self.grasp_wait_timeout_sec,
                ),
            ],
        )
        return SequenceNode(
            "RootContextPrepareTree",
            [
                ConditionNode("JointStatesReady", self._has_valid_joint_state),
                OneShotActionNode("EnsureHome", self._ensure_home),
                ConditionNode("RobotAtHome", self._is_in_home),
                OneShotActionNode("ClearPreviousGrasps", self._clear_grasp_cache_for_selection),
                OneShotActionNode("ApplySamVlmPrompt", self._apply_sam_vlm_prompt),
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
        self._clear_failure()
        while rclpy.ok():
            status = root.tick()
            if status == BTStatus.SUCCESS:
                # Preparacion ok: limpiar cualquier fallo latcheado previo.
                self._publish_skill_failure("", "", "preparacion completada")
                return True
            if status == BTStatus.FAILURE:
                # Si ningun paso publico un fallo estructurado (p.ej. fallo de
                # precondicion home/joint_states), publicar un generico.
                if not self._failure_published_this_cycle:
                    self._set_failure(
                        "execution_failed", "execution", self.last_failure_reason
                    )
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
            self.planning_ready = False
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

    def _handle_context_prepare_bt_cycle(self, request, response):
        del request

        if self.workflow_active:
            response.success = False
            response.message = "Ya hay una accion BT en ejecucion"
            return response

        self.workflow_active = True
        root = self._build_context_prepare_tree()
        try:
            self.planning_ready = False
            self.selection_done_for_cycle = False
            ok = self._run_tree_until_done(
                root, "BT: iniciando preparacion desde contexto VLM ya resuelto."
            )
            if ok:
                self.planning_ready = True
                self.selection_done_for_cycle = True
                self.planning_summary = (
                    "Preparacion desde contexto completada: SAM2 aplicado "
                    "y grasps frescos disponibles"
                )
                response.success = True
                response.message = self.planning_summary
            else:
                self.planning_ready = False
                self.planning_summary = (
                    f"BT fallo durante la preparacion desde contexto: {self.last_failure_reason}"
                )
                response.success = False
                response.message = self.planning_summary
            return response
        finally:
            root.reset()
            self.workflow_active = False

    def _handle_scan_prepare_bt_cycle(self, request, response):
        del request

        if self.workflow_active:
            response.success = False
            response.message = "Ya hay una accion BT en ejecucion"
            return response

        self.workflow_active = True
        root = self._build_scan_prepare_tree()
        try:
            self.planning_ready = False
            self.selection_done_for_cycle = False
            ok = self._run_tree_until_done(
                root, "BT: iniciando preparacion con scanner 3D desde home."
            )
            if ok:
                self.planning_ready = True
                self.planning_summary = (
                    "Preparacion con scanner completada: nube 3D escaneada y grasps frescos disponibles"
                )
                response.success = True
                response.message = self.planning_summary
            else:
                self.planning_ready = False
                self.planning_summary = (
                    f"BT fallo durante la preparacion con scanner: {self.last_failure_reason}"
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
                "No hay una preparacion lista. Primero llama a prepare_bt_grasp_cycle "
                "prepare_context_bt_grasp_cycle o prepare_scan_bt_grasp_cycle."
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
                self.planning_summary = (
                    f"Ejecucion BT fallida: {self.last_cached_execution_message}"
                )
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

    def _prepare_clean_table_object(self, use_scan: bool) -> bool:
        if use_scan:
            root = (
                self._build_scan_vlm_select_next_prepare_tree()
                if self.clean_table_use_vlm
                else self._build_scan_prepare_tree()
            )
            label = (
                "BT Limpiar la Mesa: preparando objeto con scanner."
                if not self.clean_table_use_vlm
                else "BT Limpiar la Mesa: preparando objeto con VLM select_next + scanner."
            )
        else:
            root = (
                self._build_vlm_select_next_prepare_tree()
                if self.clean_table_use_vlm
                else self._build_prepare_tree()
            )
            label = (
                "BT Limpiar la Mesa: preparando objeto."
                if not self.clean_table_use_vlm
                else "BT Limpiar la Mesa: preparando objeto con VLM select_next."
            )

        try:
            self.planning_ready = False
            self.selection_done_for_cycle = False
            ok = self._run_tree_until_done(root, label)
            self.planning_ready = ok
            return ok
        finally:
            root.reset()

    def _execute_pick_for_clean_table(self) -> bool:
        ok, _ = self._call_trigger(
            self.execute_pick_client,
            self.execute_pick_service_name,
            timeout_sec=self.execution_timeout_sec,
            label="pick de Limpiar la Mesa",
        )
        if not ok and not self._failure_published_this_cycle:
            self._set_failure("execution_failed", "execution", self.last_failure_reason)
        return ok

    def _execute_place_for_clean_table(self) -> bool:
        ok, _ = self._call_trigger(
            self.execute_place_client,
            self.execute_place_service_name,
            timeout_sec=self.execution_timeout_sec,
            label="place de Limpiar la Mesa",
        )
        if not ok and not self._failure_published_this_cycle:
            self._set_failure("place_failed", "place", self.last_failure_reason)
        return ok

    def _go_home_after_clean_table_step(self, reason: str) -> bool:
        ok, _ = self._call_trigger(
            self.go_home_client,
            self.go_home_service_name,
            timeout_sec=90.0,
            label=reason,
        )
        if not ok:
            self.get_logger().warn(
                f"BT Limpiar la Mesa: no se pudo retornar a HOME tras {reason}."
            )
        return ok

    def _clean_table_selection_means_done(self) -> bool:
        if self.last_failure_stage not in ("perception", "segmentation"):
            return False

        reason = (self.last_failure_reason or "").lower()
        no_object_markers = (
            "sin objeto",
            "sin objetos",
            "no hay objeto",
            "no hay objetos",
            "no object",
            "no objects",
            "object_not_found",
            "objeto no encontrado",
            "no se detect",
            "mesa limpia",
            "table clean",
        )
        if any(marker in reason for marker in no_object_markers):
            return True

        if self.clean_table_treat_selection_failure_as_done:
            hard_failure_markers = (
                "servicio",
                "service",
                "timeout",
                "exception",
                "traceback",
                "cuda",
                "oom",
                "vrAM".lower(),
            )
            return not any(marker in reason for marker in hard_failure_markers)

        return False

    def _handle_clean_table(self, request, response):
        del request

        if self.workflow_active:
            response.success = False
            response.message = "Ya hay una accion BT en ejecucion"
            return response

        self.workflow_active = True
        cleaned_count = 0
        discarded_count = 0
        try:
            if self.clean_table_use_vlm:
                self._reset_vlm_handled_objects()

            self.get_logger().info(
                "BT Limpiar la Mesa: iniciando ciclo iterativo "
                f"(max_cycles={self.clean_table_max_cycles}, "
                f"use_vlm={self.clean_table_use_vlm}, "
                f"scan_on_no_grasps={self.clean_table_scan_on_no_grasps})."
            )

            for cycle_index in range(1, self.clean_table_max_cycles + 1):
                self.get_logger().info(
                    f"BT Limpiar la Mesa: ciclo {cycle_index}/"
                    f"{self.clean_table_max_cycles}."
                )

                if not self._prepare_clean_table_object(use_scan=False):
                    if self._clean_table_selection_means_done():
                        response.success = True
                        response.message = (
                            "Mesa limpia: no se detectaron mas objetos "
                            f"(retirados={cleaned_count}, descartados={discarded_count})."
                        )
                        return response

                    if (
                        self.clean_table_scan_on_no_grasps
                        and self.last_failure_type == "no_grasps_generated"
                    ):
                        self.get_logger().warn(
                            "BT Limpiar la Mesa: no hubo grasps; intentando scanner."
                        )
                        if not self._prepare_clean_table_object(use_scan=True):
                            if self.last_failure_type == "no_grasps_generated":
                                discarded_count += 1
                                self.get_logger().warn(
                                    "BT Limpiar la Mesa: objeto descartado tras "
                                    "fallar generacion de grasps con scanner."
                                )
                                self._go_home_after_clean_table_step(
                                    "descarte de objeto sin grasps"
                                )
                                continue
                            response.success = False
                            response.message = (
                                "BT Limpiar la Mesa fallo durante preparacion "
                                f"con scanner: {self.last_failure_reason}"
                            )
                            return response
                    else:
                        response.success = False
                        response.message = (
                            "BT Limpiar la Mesa fallo durante preparacion: "
                            f"{self.last_failure_reason}"
                        )
                        return response

                if not self._execute_pick_for_clean_table():
                    self.get_logger().warn(
                        "BT Limpiar la Mesa: pick fallo; retornando a HOME y "
                        "reiniciando ciclo."
                    )
                    self._go_home_after_clean_table_step("fallo de pick")
                    continue

                if not self._execute_place_for_clean_table():
                    self._go_home_after_clean_table_step("fallo de place")
                    response.success = False
                    response.message = (
                        "BT Limpiar la Mesa fallo durante place: "
                        f"{self.last_failure_reason}"
                    )
                    return response

                cleaned_count += 1
                self._go_home_after_clean_table_step("place completado")

            response.success = False
            response.message = (
                "BT Limpiar la Mesa alcanzo el maximo de ciclos sin confirmar "
                f"mesa limpia (retirados={cleaned_count}, "
                f"descartados={discarded_count}, max_cycles={self.clean_table_max_cycles})."
            )
            return response
        finally:
            self.planning_ready = False
            self.selection_done_for_cycle = False
            self.workflow_active = False


    # ------------------------------------------------------------------
    # VLM — comunicación HTTP con el servidor Flask (sin control de Docker)
    #
    # El servidor VLM (Gemma) debe estar levantado externamente por el
    # usuario / docker compose. Este nodo NO inicia, detiene ni inspecciona
    # contenedores Docker: solo habla HTTP/Flask con el servidor.
    # ------------------------------------------------------------------

    def _get_gpu_free_gb(self) -> float:
        """Return actual GPU free memory (GiB) via nvidia-smi (host view).

        PyTorch's cuda_allocated_gb inside a Docker container can report near 0
        even when the CUDA context still holds gigabytes on the device.  This
        queries the driver directly so the result is independent of container
        boundaries.
        """
        try:
            result = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-gpu=memory.free",
                    "--format=csv,noheader,nounits",
                ],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if result.returncode == 0:
                lines = result.stdout.strip().splitlines()
                if lines:
                    return float(lines[0].strip()) / 1024.0  # MiB → GiB
        except Exception as exc:
            self.get_logger().debug(f"BT: nvidia-smi query failed: {exc}")
        return 0.0

    def _wait_vlm_health(self) -> bool:
        self.get_logger().info("BT: verificando servidor VLM por HTTP")
        deadline = time.monotonic() + self.vlm_health_timeout_sec
        poll_interval = 2.0

        while time.monotonic() < deadline:
            try:
                req = urllib.request.Request(self.vlm_health_url)
                with urllib.request.urlopen(req, timeout=5) as resp:
                    if resp.status == 200:
                        data = json.loads(resp.read().decode())
                        # El servidor responde aunque el modelo no esté
                        # cargado (se carga bajo demanda en /infer_image).
                        if data.get("success") or data.get("status") == "ok":
                            self.get_logger().info("BT: servidor VLM listo")
                            return True
            except Exception:
                pass
            time.sleep(poll_interval)

        self.last_failure_reason = (
            f"VLM health no respondió en {self.vlm_health_timeout_sec}s "
            f"en {self.vlm_health_url}"
        )
        self.get_logger().error(f"BT: {self.last_failure_reason}")
        return False

    def _release_vlm(self) -> bool:
        return True

    def _wait_vlm_vram_released(self) -> bool:
        return True

    def _release_perception_models(self):
        pass

    def _load_sam_model(self) -> bool:
        self.get_logger().info("BT: recargando modelo SAM2 tras el VLM")
        ok, _ = self._call_trigger(
            self.sam_load_client,
            self.sam_load_service_name,
            timeout_sec=120.0,
            label="carga de modelo SAM2",
        )
        return ok

    def _apply_sam_vlm_prompt(self) -> bool:
        self.get_logger().info("BT: aplicando prompt VLM en SAM2 (/sam2/apply_vlm_prompt)")
        ok, _ = self._call_trigger(
            self.sam_apply_vlm_prompt_client,
            self.sam_apply_vlm_prompt_service_name,
            timeout_sec=120.0,
            label="aplicacion de prompt SAM2",
        )
        return ok

    def _ensure_graspgen_loaded(self) -> bool:
        self.get_logger().info("BT: recargando modelo GraspGen")
        ok, _ = self._call_trigger(
            self.graspgen_load_client,
            self.graspgen_load_service_name,
            timeout_sec=120.0,
            label="carga de modelo GraspGen",
        )
        return ok

    def _call_vlm_with_lifecycle(self) -> bool:
        if self.selection_done_for_cycle:
            self.get_logger().info(
                "BT: la seleccion VLM ya fue realizada en este ciclo de preparacion."
            )
            return True

        # 1-2. Liberar SAM2/GraspGen de la GPU; 3. esperar a que CUDA libere.
        self._release_perception_models()

        # 4. Verificar que el servidor VLM responde antes de inferir.
        if not self._wait_vlm_health():
            return False

        # 5. Inferencia semantica VLM (genera /tmp/vlm_prompt/sam2_vlm_prompt.json).
        ok = False
        try:
            self.get_logger().info(
                "BT: llamando inferencia VLM via /vlm_prompt/select_object"
            )
            ok, _ = self._call_trigger(
                self.vlm_select_client,
                self.vlm_select_service_name,
                timeout_sec=120.0,
                label="seleccion VLM semantica",
            )
        finally:
            # 6. Liberar siempre la GPU del modelo VLM, incluso si la seleccion
            # fallo. Un fallo de /release solo genera warning.
            self._release_vlm()

        if not ok:
            return False

        # 6b. Esperar a que la VRAM este realmente libre (host + contenedor).
        #     Si no se libera, abortar: cargar SAM2 con la GPU llena causaria OOM.
        if not self._wait_vlm_vram_released():
            self._set_failure(
                "segmentation_failed",
                "segmentation",
                "VLM no libero suficiente VRAM; SAM2 no puede cargarse sin CUDA OOM",
            )
            return False

        # 7. Recargar SAM2 ahora que la VRAM del VLM esta libre.
        if not self._load_sam_model():
            return False

        # 8. Aplicar el prompt VLM en SAM2 (la inferencia VLM ya no lo hace).
        if not self._apply_sam_vlm_prompt():
            return False

        # 9. GraspGen se recarga justo antes de su inferencia (CallGraspGen /
        #    LoadGraspGen en el arbol), para no competir con SAM2 por VRAM.
        self.selection_done_for_cycle = True
        self.get_logger().info("Continuing with SAM2/GraspGen")
        return True

    def _reset_vlm_handled_objects(self) -> bool:
        ok, _ = self._call_trigger(
            self.vlm_reset_handled_client,
            self.vlm_reset_handled_service_name,
            timeout_sec=5.0,
            label="reset objetos VLM manejados",
        )
        return ok

    def _call_vlm_select_next_with_lifecycle(self) -> bool:
        if self.selection_done_for_cycle:
            self.get_logger().info(
                "BT: la seleccion VLM-next ya fue realizada en este ciclo."
            )
            return True

        self._release_perception_models()

        # Esperar a que SAM2/GraspGen liberen la VRAM antes de cargar el VLM.
        if not self._wait_vlm_vram_released():
            self._set_failure(
                "segmentation_failed",
                "segmentation",
                "SAM2/GraspGen no liberaron suficiente VRAM para cargar VLM",
            )
            return False

        if not self._wait_vlm_health():
            return False

        ok = False
        msg = ""
        try:
            self.get_logger().info(
                "BT: llamando select_next_object via /vlm_prompt/select_next_object"
            )
            ok, msg = self._call_trigger(
                self.vlm_select_next_client,
                self.vlm_select_next_service_name,
                timeout_sec=120.0,
                label="seleccion VLM siguiente objeto",
            )
        finally:
            self._release_vlm()

        if not ok:
            low = (msg or "").lower()
            if "mesa limpia" in low or "table clean" in low or "done" in low:
                self._set_failure("segmentation_failed", "segmentation", "mesa limpia")
            return False

        if not self._wait_vlm_vram_released():
            self._set_failure(
                "segmentation_failed",
                "segmentation",
                "VLM no libero suficiente VRAM; SAM2 no puede cargarse sin CUDA OOM",
            )
            return False

        if not self._load_sam_model():
            return False

        if not self._apply_sam_vlm_prompt():
            return False

        self.selection_done_for_cycle = True
        self.get_logger().info("BT: select_next_object completado, continuando con SAM2/GraspGen")
        return True

    # ------------------------------------------------------------------
    # VLM prepare trees
    # ------------------------------------------------------------------

    def _build_vlm_prepare_tree(self) -> BTNode:
        planning_attempt_sequence = SequenceNode(
            "GraspPlanningSequence",
            [
                OneShotActionNode("CallGraspGen", self._call_grasp_generation),
                WaitForGraspsNode(
                    "WaitForFreshGrasps",
                    self._has_fresh_grasps,
                    lambda: self.grasp_wait_timeout_sec,
                ),
            ],
        )
        return SequenceNode(
            "RootVlmPrepareTree",
            [
                ConditionNode("JointStatesReady", self._has_valid_joint_state),
                OneShotActionNode("EnsureHome", self._ensure_home),
                ConditionNode("RobotAtHome", self._is_in_home),
                OneShotActionNode("ClearPreviousGrasps", self._clear_grasp_cache_for_selection),
                OneShotActionNode("CallVlm", self._call_vlm_with_lifecycle),
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

    def _build_scan_vlm_prepare_tree(self) -> BTNode:
        return SequenceNode(
            "RootScanVlmPrepareTree",
            [
                ConditionNode("JointStatesReady", self._has_valid_joint_state),
                OneShotActionNode("EnsureHome", self._ensure_home),
                ConditionNode("RobotAtHome", self._is_in_home),
                OneShotActionNode("ClearPreviousGrasps", self._clear_grasp_cache_for_selection),
                OneShotActionNode("CallVlm", self._call_vlm_with_lifecycle),
                OneShotActionNode("LoadGraspGen", self._ensure_graspgen_loaded),
                OneShotActionNode("OctoHomeCloud", self._capture_octomap_home_cloud),
                OneShotActionNode("ClearOctomap", self._clear_octomap),
                OneShotActionNode("CallCaptureScanner", self._call_capture_scan),
                WaitForGraspsNode(
                    "WaitForScanInference",
                    self._has_fresh_grasps,
                    lambda: self.scan_wait_timeout_sec,
                ),
            ],
        )

    def _build_vlm_select_next_prepare_tree(self) -> BTNode:
        planning_attempt_sequence = SequenceNode(
            "GraspPlanningSequenceNext",
            [
                OneShotActionNode("CallGraspGen", self._call_grasp_generation),
                WaitForGraspsNode(
                    "WaitForFreshGrasps",
                    self._has_fresh_grasps,
                    lambda: self.grasp_wait_timeout_sec,
                ),
            ],
        )
        return SequenceNode(
            "RootVlmSelectNextPrepareTree",
            [
                ConditionNode("JointStatesReady", self._has_valid_joint_state),
                OneShotActionNode("EnsureHome", self._ensure_home),
                ConditionNode("RobotAtHome", self._is_in_home),
                OneShotActionNode("ClearPreviousGrasps", self._clear_grasp_cache_for_selection),
                OneShotActionNode("CallVlmSelectNext", self._call_vlm_select_next_with_lifecycle),
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

    def _build_scan_vlm_select_next_prepare_tree(self) -> BTNode:
        return SequenceNode(
            "RootScanVlmSelectNextPrepareTree",
            [
                ConditionNode("JointStatesReady", self._has_valid_joint_state),
                OneShotActionNode("EnsureHome", self._ensure_home),
                ConditionNode("RobotAtHome", self._is_in_home),
                OneShotActionNode("ClearPreviousGrasps", self._clear_grasp_cache_for_selection),
                OneShotActionNode("CallVlmSelectNext", self._call_vlm_select_next_with_lifecycle),
                OneShotActionNode("LoadGraspGen", self._ensure_graspgen_loaded),
                OneShotActionNode("OctoHomeCloud", self._capture_octomap_home_cloud),
                OneShotActionNode("ClearOctomap", self._clear_octomap),
                OneShotActionNode("CallCaptureScanner", self._call_capture_scan),
                WaitForGraspsNode(
                    "WaitForScanInference",
                    self._has_fresh_grasps,
                    lambda: self.scan_wait_timeout_sec,
                ),
            ],
        )

    # ------------------------------------------------------------------
    # VLM service handlers
    # ------------------------------------------------------------------

    def _handle_vlm_prepare_bt_cycle(self, request, response):
        del request

        if self.workflow_active:
            response.success = False
            response.message = "Ya hay una accion BT en ejecucion"
            return response

        self.workflow_active = True
        root = self._build_vlm_prepare_tree()
        try:
            self.planning_ready = False
            self.selection_done_for_cycle = False
            ok = self._run_tree_until_done(
                root, "BT: iniciando preparacion VLM desde home."
            )
            if ok:
                self.planning_ready = True
                self.planning_summary = (
                    "Preparacion VLM completada: objeto seleccionado con VLM+SAM2 "
                    "y grasps frescos disponibles"
                )
                response.success = True
                response.message = self.planning_summary
            else:
                self.planning_ready = False
                self.planning_summary = (
                    f"BT fallo durante la preparacion VLM: {self.last_failure_reason}"
                )
                response.success = False
                response.message = self.planning_summary
            return response
        finally:
            root.reset()
            self.workflow_active = False

    def _handle_scan_vlm_prepare_bt_cycle(self, request, response):
        del request

        if self.workflow_active:
            response.success = False
            response.message = "Ya hay una accion BT en ejecucion"
            return response

        self.workflow_active = True
        root = self._build_scan_vlm_prepare_tree()
        try:
            self.planning_ready = False
            self.selection_done_for_cycle = False
            ok = self._run_tree_until_done(
                root, "BT: iniciando preparacion VLM con scanner 3D desde home."
            )
            if ok:
                self.planning_ready = True
                self.planning_summary = (
                    "Preparacion VLM+scanner completada: nube 3D escaneada "
                    "y grasps frescos disponibles"
                )
                response.success = True
                response.message = self.planning_summary
            else:
                self.planning_ready = False
                self.planning_summary = (
                    f"BT fallo durante la preparacion VLM+scanner: {self.last_failure_reason}"
                )
                response.success = False
                response.message = self.planning_summary
            return response
        finally:
            root.reset()
            self.workflow_active = False


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
