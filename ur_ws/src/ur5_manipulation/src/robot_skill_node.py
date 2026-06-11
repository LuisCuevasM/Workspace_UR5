#!/usr/bin/env python3
"""Skills de manipulacion para el orquestador LangGraph (via rosbridge).

Expone los servicios del contrato `ROS2_INTERFACES.md` con tipo
`ur5_manipulation/RunSkill`:

    /robot/grasp       acercarse + cerrar gripper (sin levantar)
    /robot/pick        grasp + levantar a retract
    /robot/place       llevar el objeto ya agarrado al destino y soltar
    /robot/pick_place  pick + place
    /robot/push        declarada, NO implementada -> skill_not_available
    /robot/handover    declarada, NO implementada -> skill_not_available

Orquesta los servicios de bajo nivel existentes (todos std_srvs/Trigger):

    grasp/pick prepare  -> /ur5/prepare_vlm_bt_grasp_cycle  (VLM + SAM2 + GraspGen)
                           o, si la request llega SIN contexto VLM
                           (vlm_context_json vacio o sin box),
                           /ur5/prepare_bt_grasp_cycle (seleccion SAM2 sin VLM)
    pick execute        -> /ur5/execute_pick
    place execute       -> /ur5/execute_place
    limpiar cache       -> /ur5/clear_grasp_cache

Semantica OBLIGATORIA de flags (contrato):
    clear_cached_grasps=true  -> invalida top_grasps_cached (clear_grasp_cache)
                                 y re-ejecuta GraspGen sobre segmentacion fresca.
    force_new_perception=true -> re-publica el prompt VLM a SAM2 para re-segmentar
                                 antes de preparar el ciclo.
El nodo respeta ambos flags SIEMPRE, aunque haya cache valida.

Como los servicios downstream solo devuelven (success, message), este nodo
traduce el resultado a (stage, failure_type) del contrato con una heuristica
sobre el texto del mensaje, ademas de la etapa donde se produjo el fallo.
"""

import json
import time

import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile

from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import CameraInfo, Image

from ur5_manipulation.srv import RunSkill


# Clasificacion heuristica del mensaje downstream -> (failure_type, stage).
# FALLBACK unicamente: la fuente confiable es el fallo estructurado publicado en
# /ur5/skill_failure por el BT node y move_group_client (ver _resolve_failure).
# Esta heuristica solo se usa si ese fallo estructurado no llega a tiempo.
# El orden importa: la primera coincidencia gana.
_MESSAGE_RULES = [
    (("object_not_found", "objeto no encontrado", "no se encontro el objeto",
      "no se encontró el objeto", "no hay objeto", "objeto no visible",
      "objeto no presente"), "object_not_found", "perception"),
    (("segment", "sam2", "sam ", "mascara", "máscara", "sin nube"), "segmentation_failed", "segmentation"),
    (("sin grasps", "no grasps", "ningun grasp", "ningún grasp", "graspgen no", "no candidat"), "no_grasps_generated", "grasp_generation"),
    (("colision", "colisión", "collision", "filtrad"), "no_collision_free_grasps", "collision_filter"),
    (("ik", "cinemat", "cinemát", "planner", "plan fallo", "plan fallido", "singular", "no se encontro plan", "no se encontró plan"), "ik_failed", "ik"),
    (("slip", "se solto", "se soltó", "deslic"), "object_slipped", "execution"),
    (("gripper", "pinza"), "gripper_failed", "gripper"),
    (("deposit", "place", "soltar"), "place_failed", "place"),
    (("timeout", "trajector", "trayector", "ejecu", "execution"), "execution_failed", "execution"),
]


def _classify(message, default_failure, default_stage):
    low = (message or "").lower()
    for keywords, failure_type, stage in _MESSAGE_RULES:
        if any(k in low for k in keywords):
            return failure_type, stage
    return default_failure, default_stage


class RobotSkillNode(Node):
    def __init__(self):
        super().__init__("robot_skill_node")
        self.cb_group = ReentrantCallbackGroup()

        # --- Servicios downstream ---
        self.prepare_service = self.declare_parameter(
            "prepare_service", "/ur5/prepare_vlm_bt_grasp_cycle"
        ).value
        self.prepare_no_vlm_service = self.declare_parameter(
            "prepare_no_vlm_service", "/ur5/prepare_bt_grasp_cycle"
        ).value
        self.execute_pick_service = self.declare_parameter(
            "execute_pick_service", "/ur5/execute_pick"
        ).value
        self.execute_place_service = self.declare_parameter(
            "execute_place_service", "/ur5/execute_place"
        ).value
        self.clear_cache_service = self.declare_parameter(
            "clear_cache_service", "/ur5/clear_grasp_cache"
        ).value
        self.sam_prompt_topic = self.declare_parameter(
            "sam_prompt_topic", "/sam2/vlm_prompt"
        ).value
        self.cam_info_topic = self.declare_parameter(
            "cam_info_topic", "/camera/camera/aligned_depth_to_color/camera_info"
        ).value
        self.skill_failure_topic = self.declare_parameter(
            "skill_failure_topic", "/ur5/skill_failure"
        ).value
        self.structured_failure_wait_sec = float(
            self.declare_parameter("structured_failure_wait_sec", 1.0).value
        )

        # --- Timeouts ---
        self.service_wait_timeout = float(
            self.declare_parameter("service_wait_timeout_sec", 5.0).value
        )
        self.prepare_timeout = float(
            self.declare_parameter("prepare_timeout_sec", 120.0).value
        )
        self.execute_timeout = float(
            self.declare_parameter("execute_timeout_sec", 120.0).value
        )
        self.sam_settle_sec = float(
            self.declare_parameter("sam_settle_sec", 1.0).value
        )

        # --- Estado camara (para denormalizar prompts VLM a pixeles) ---
        self._cam_width = None
        self._cam_height = None
        self.create_subscription(
            CameraInfo, self.cam_info_topic, self._on_info, 10,
            callback_group=self.cb_group,
        )

        # --- Clientes ---
        self.prepare_client = self.create_client(
            Trigger, self.prepare_service, callback_group=self.cb_group
        )
        self.prepare_no_vlm_client = self.create_client(
            Trigger, self.prepare_no_vlm_service, callback_group=self.cb_group
        )
        self.pick_client = self.create_client(
            Trigger, self.execute_pick_service, callback_group=self.cb_group
        )
        self.place_client = self.create_client(
            Trigger, self.execute_place_service, callback_group=self.cb_group
        )
        self.clear_cache_client = self.create_client(
            Trigger, self.clear_cache_service, callback_group=self.cb_group
        )
        self.prompt_pub = self.create_publisher(String, self.sam_prompt_topic, 10)

        # Fallo estructurado latcheado desde los nodos downstream (BT + move_group).
        # Es la fuente confiable de failure_type/stage; _classify es solo fallback.
        self._failure_lock = threading.Lock()
        self._latest_failure = None
        failure_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, self.skill_failure_topic, self._on_skill_failure, failure_qos,
            callback_group=self.cb_group,
        )

        # --- Servicios expuestos ---
        self._make_skill_service("/robot/grasp", self._handle_grasp)
        self._make_skill_service("/robot/pick", self._handle_pick)
        self._make_skill_service("/robot/place", self._handle_place)
        self._make_skill_service("/robot/pick_place", self._handle_pick_place)
        self._make_skill_service("/robot/push", self._handle_unavailable)
        self._make_skill_service("/robot/handover", self._handle_unavailable)

        self.get_logger().info(
            "robot_skill_node listo: /robot/grasp, /robot/pick, /robot/place, "
            "/robot/pick_place, /robot/push (stub), /robot/handover (stub)"
        )

    def _make_skill_service(self, name, handler):
        self.create_service(RunSkill, name, handler, callback_group=self.cb_group)

    def _on_info(self, msg):
        self._cam_width = int(msg.width)
        self._cam_height = int(msg.height)

    def _on_skill_failure(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._failure_lock:
            self._latest_failure = data

    def _resolve_failure(self, message, default_failure, default_stage, call_start):
        """failure_type/stage confiable: prioriza el fallo estructurado publicado
        por los nodos downstream (>= call_start); si no llega, cae a la heuristica.
        """
        deadline = time.monotonic() + self.structured_failure_wait_sec
        while True:
            with self._failure_lock:
                f = self._latest_failure
            if (
                f is not None
                and not f.get("ok", True)
                and float(f.get("stamp", 0.0)) >= call_start - 0.5
            ):
                return (
                    f.get("failure_type") or default_failure,
                    f.get("stage") or default_stage,
                    f.get("detail", ""),
                )
            if time.monotonic() >= deadline:
                break
            time.sleep(0.05)
        failure_type, stage = _classify(message, default_failure, default_stage)
        return failure_type, stage, message

    # ------------------------------------------------------------------
    # Utilidades downstream
    # ------------------------------------------------------------------
    def _call_trigger(self, client, name, timeout):
        if not client.wait_for_service(timeout_sec=self.service_wait_timeout):
            return False, f"Servicio '{name}' no disponible"
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return False, f"Timeout en '{name}'"
        resp = future.result()
        if resp is None:
            return False, f"Sin respuesta de '{name}'"
        return bool(resp.success), resp.message or ""

    @staticmethod
    def _has_vlm_context(req):
        """True si la request trae contexto VLM usable (con box normalizado)."""
        if not req.vlm_context_json:
            return False
        try:
            ctx = json.loads(req.vlm_context_json)
        except json.JSONDecodeError:
            return False
        box = ctx.get("object_box_xyxy_norm") if isinstance(ctx, dict) else None
        return bool(box) and len(box) == 4

    def _apply_flags(self, req, detail, use_vlm):
        """Honra clear_cached_grasps y force_new_perception ANTES de preparar."""
        if req.clear_cached_grasps:
            ok, msg = self._call_trigger(
                self.clear_cache_client, self.clear_cache_service, self.execute_timeout
            )
            detail["clear_cached_grasps"] = {"ok": ok, "message": msg}
            self.get_logger().info(f"clear_grasp_cache -> {ok}: {msg}")

        if req.force_new_perception:
            if use_vlm:
                ok, msg = self._push_vlm_prompt(req.vlm_context_json)
                detail["force_new_perception"] = {"ok": ok, "message": msg}
                self.get_logger().info(f"re-segmentacion SAM2 -> {ok}: {msg}")
                # Dar tiempo a SAM2 a procesar el nuevo prompt antes de preparar.
                time.sleep(self.sam_settle_sec)
            else:
                # Sin contexto VLM no hay prompt que re-publicar; el ciclo de
                # preparacion no-VLM re-segmenta por si mismo en cada llamada.
                msg = "sin contexto VLM: el prepare no-VLM re-segmenta solo"
                detail["force_new_perception"] = {"ok": True, "message": msg}
                self.get_logger().info(msg)

    def _push_vlm_prompt(self, vlm_context_json):
        """Publica el prompt VLM (box/puntos) a SAM2 en pixeles para re-segmentar."""
        if not vlm_context_json:
            return False, "vlm_context_json vacio"
        try:
            ctx = json.loads(vlm_context_json)
        except json.JSONDecodeError as exc:
            return False, f"vlm_context_json invalido: {exc}"

        box = ctx.get("object_box_xyxy_norm")
        if not box or len(box) != 4:
            return False, "vlm_context_json sin object_box_xyxy_norm"

        if self._cam_width is None or self._cam_height is None:
            return False, f"sin camera_info en '{self.cam_info_topic}'"

        w, h = self._cam_width, self._cam_height
        box_px = [box[0] * w, box[1] * h, box[2] * w, box[3] * h]
        coords, labels = [], []
        for p in ctx.get("positive_points_xy_norm", []) or []:
            coords.append([p[0] * w, p[1] * h])
            labels.append(1)
        for p in ctx.get("negative_points_xy_norm", []) or []:
            coords.append([p[0] * w, p[1] * h])
            labels.append(0)

        prompt = {
            "box_xyxy": box_px,
            "point_coords": coords,
            "point_labels": labels,
            "target_object": ctx.get("name", ""),
            "selected_safe_part": ctx.get("selected_safe_part", ""),
        }
        self.prompt_pub.publish(String(data=json.dumps(prompt)))
        return True, "prompt VLM publicado a SAM2"

    @staticmethod
    def _mask_id_from_ctx(vlm_context_json):
        try:
            return json.loads(vlm_context_json or "{}").get("mask_id", "")
        except json.JSONDecodeError:
            return ""

    def _respond(self, response, success, stage, failure_type, mask_id, detail,
                 grasp_id=""):
        response.success = success
        response.stage = stage
        response.failure_type = failure_type
        response.grasp_id = grasp_id
        response.mask_id = mask_id
        response.detail_json = json.dumps(detail)
        return response

    # ------------------------------------------------------------------
    # Pasos del pipeline
    # ------------------------------------------------------------------
    def _do_prepare(self, req, detail):
        """SAM2 + GraspGen. Devuelve (ok, failure_type, stage, message).

        Con contexto VLM usa el ciclo VLM; sin el, el ciclo no-VLM
        (seleccion SAM2 directa), para operar sin el servidor VLM.
        """
        use_vlm = self._has_vlm_context(req)
        detail["prepare_mode"] = "vlm" if use_vlm else "no_vlm"
        self._apply_flags(req, detail, use_vlm)
        client, service_name = (
            (self.prepare_client, self.prepare_service)
            if use_vlm
            else (self.prepare_no_vlm_client, self.prepare_no_vlm_service)
        )
        self.get_logger().info(f"prepare via '{service_name}' (modo {detail['prepare_mode']})")
        call_start = time.time()
        ok, msg = self._call_trigger(client, service_name, self.prepare_timeout)
        detail["prepare"] = {"ok": ok, "message": msg}
        if ok:
            return True, "", "grasp_generation", msg
        failure_type, stage, det = self._resolve_failure(
            msg, "no_grasps_generated", "grasp_generation", call_start
        )
        detail["prepare"]["failure_detail"] = det
        return False, failure_type, stage, msg

    def _do_execute(self, client, name, detail, key, default_failure, default_stage):
        call_start = time.time()
        ok, msg = self._call_trigger(client, name, self.execute_timeout)
        detail[key] = {"ok": ok, "message": msg}
        if ok:
            return True, "", default_stage, msg
        failure_type, stage, det = self._resolve_failure(
            msg, default_failure, default_stage, call_start
        )
        detail[key]["failure_detail"] = det
        return False, failure_type, stage, msg

    # ------------------------------------------------------------------
    # Skills
    # ------------------------------------------------------------------
    def _handle_grasp(self, req, response):
        detail = {"skill": "grasp", "attempt_number": req.attempt_number,
                  "object_name": req.object_name}
        mask_id = self._mask_id_from_ctx(req.vlm_context_json)
        ok, ft, stage, _ = self._do_prepare(req, detail)
        if not ok:
            return self._respond(response, False, stage, ft, mask_id, detail)
        # grasp = preparar grasps (sin mover el brazo).
        return self._respond(response, True, "grasp_generation", "", mask_id, detail,
                             grasp_id=f"grasp_{req.attempt_number}")

    def _handle_pick(self, req, response):
        detail = {"skill": "pick", "attempt_number": req.attempt_number,
                  "object_name": req.object_name}
        mask_id = self._mask_id_from_ctx(req.vlm_context_json)
        grasp_id = f"grasp_{req.attempt_number}"

        ok, ft, stage, _ = self._do_prepare(req, detail)
        if not ok:
            return self._respond(response, False, stage, ft, mask_id, detail, grasp_id)

        ok, ft, stage, _ = self._do_execute(
            self.pick_client, self.execute_pick_service, detail, "execute_pick",
            "execution_failed", "execution",
        )
        if not ok:
            return self._respond(response, False, stage, ft, mask_id, detail, grasp_id)

        return self._respond(response, True, "done", "", mask_id, detail, grasp_id)

    def _handle_place(self, req, response):
        detail = {"skill": "place", "attempt_number": req.attempt_number,
                  "object_name": req.object_name,
                  "target_location": req.target_location}
        mask_id = self._mask_id_from_ctx(req.vlm_context_json)

        ok, ft, stage, msg = self._do_execute(
            self.place_client, self.execute_place_service, detail, "execute_place",
            "place_failed", "place",
        )
        if not ok:
            # El contrato exige reportar si el objeto sigue agarrado para que el
            # orquestador decida entre reintentar place o un pick_place completo.
            # Sin sensor de validacion aqui asumimos que sigue agarrado salvo
            # evidencia textual de lo contrario.
            still = "se solto" not in msg.lower() and "se soltó" not in msg.lower()
            detail["object_still_grasped"] = bool(still)
            return self._respond(response, False, stage, ft, mask_id, detail)

        return self._respond(response, True, "done", "", mask_id, detail)

    def _handle_pick_place(self, req, response):
        detail = {"skill": "pick_place", "attempt_number": req.attempt_number,
                  "object_name": req.object_name,
                  "target_location": req.target_location}
        mask_id = self._mask_id_from_ctx(req.vlm_context_json)
        grasp_id = f"grasp_{req.attempt_number}"

        ok, ft, stage, _ = self._do_prepare(req, detail)
        if not ok:
            return self._respond(response, False, stage, ft, mask_id, detail, grasp_id)

        ok, ft, stage, _ = self._do_execute(
            self.pick_client, self.execute_pick_service, detail, "execute_pick",
            "execution_failed", "execution",
        )
        if not ok:
            return self._respond(response, False, stage, ft, mask_id, detail, grasp_id)

        ok, ft, stage, msg = self._do_execute(
            self.place_client, self.execute_place_service, detail, "execute_place",
            "place_failed", "place",
        )
        if not ok:
            still = "se solto" not in msg.lower() and "se soltó" not in msg.lower()
            detail["object_still_grasped"] = bool(still)
            return self._respond(response, False, stage, ft, mask_id, detail, grasp_id)

        return self._respond(response, True, "done", "", mask_id, detail, grasp_id)

    def _handle_unavailable(self, req, response):
        detail = {"skill": "unavailable", "object_name": req.object_name,
                  "attempt_number": req.attempt_number}
        self.get_logger().warn(
            f"Skill no implementada solicitada para '{req.object_name}'."
        )
        return self._respond(response, False, "", "skill_not_available", "", detail)


def main(args=None):
    rclpy.init(args=args)
    node = RobotSkillNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
