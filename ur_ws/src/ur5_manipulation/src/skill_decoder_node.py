#!/usr/bin/env python3
"""Decodifica skills seleccionadas por el VLM y ejecuta la skill ROS2.

Entrada esperada: salida normalizada de /ur5/query_instruction:
selected_skill, object_name, target_location y vlm_context_json.

Tambien expone /ur5/execute_instruction para probar el flujo completo con una
instruccion natural: query VLM -> decode -> /robot/<skill>.

Semantica:
  - clean_table -> /robot/clean_table
  - grasp       -> /robot/grasp con box/puntos VLM para SAM2 + GraspGen
  - pick        -> /robot/pick sin reenviar box; usa grasps cacheados
  - place       -> /robot/place hacia ubicacion predefinida
  - pick_place  -> /robot/pick_place con contexto si esta disponible
"""

import json
import re
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ur5_manipulation.srv import DecodeSkill, ExecuteInstruction, QueryInstruction, RunSkill


SKILL_TO_SERVICE = {
    "clean_table": "/robot/clean_table",
    "grasp": "/robot/grasp",
    "pick": "/robot/pick",
    "place": "/robot/place",
    "pick_place": "/robot/pick_place",
    "push": "/robot/push",
    "handover": "/robot/handover",
}

SKILL_ALIASES = {
    "clean table": "clean_table",
    "clean-the-table": "clean_table",
    "clean_the_table": "clean_table",
    "limpiar mesa": "clean_table",
    "limpiar_mesa": "clean_table",
    "limpia mesa": "clean_table",
    "limpia_mesa": "clean_table",
    "limpiar la mesa": "clean_table",
    "limpia la mesa": "clean_table",
    "pick and place": "pick_place",
    "pick-and-place": "pick_place",
    "pickplace": "pick_place",
    "pick place": "pick_place",
}


def _json_dumps(data):
    return json.dumps(data, ensure_ascii=False, sort_keys=True)


class SkillDecoderNode(Node):
    def __init__(self):
        super().__init__("skill_decoder_node")
        self.cb_group = ReentrantCallbackGroup()

        self.service_name = self.declare_parameter(
            "service_name", "/ur5/decode_skill"
        ).value
        self.execute_instruction_service_name = self.declare_parameter(
            "execute_instruction_service_name", "/ur5/execute_instruction"
        ).value
        self.query_instruction_service_name = self.declare_parameter(
            "query_instruction_service_name", "/ur5/query_instruction"
        ).value
        self.service_wait_timeout = float(
            self.declare_parameter("service_wait_timeout_sec", 5.0).value
        )
        self.skill_timeout = float(
            self.declare_parameter("skill_timeout_sec", 180.0).value
        )
        self.query_timeout = float(
            self.declare_parameter("query_timeout_sec", 180.0).value
        )

        self.skill_clients = {
            skill: self.create_client(
                RunSkill, service_name, callback_group=self.cb_group
            )
            for skill, service_name in SKILL_TO_SERVICE.items()
        }
        self.query_instruction_client = self.create_client(
            QueryInstruction,
            self.query_instruction_service_name,
            callback_group=self.cb_group,
        )

        self.create_service(
            DecodeSkill,
            self.service_name,
            self._handle_decode_skill,
            callback_group=self.cb_group,
        )
        self.create_service(
            ExecuteInstruction,
            self.execute_instruction_service_name,
            self._handle_execute_instruction,
            callback_group=self.cb_group,
        )

        self._attempt_counter = 0
        self.get_logger().info(
            "skill_decoder_node listo: "
            f"{self.service_name} y {self.execute_instruction_service_name}"
        )

    @staticmethod
    def _normalize_skill(selected_skill: str) -> str:
        text = (selected_skill or "").strip().lower()
        if not text:
            return ""

        # Acepta "robot.grasp", "/robot/grasp", "robot:grasp", etc.
        text = re.sub(r"^/?robot[./:_-]+", "", text)
        text = text.replace("-", "_")
        text = re.sub(r"\s+", " ", text).strip()
        text = SKILL_ALIASES.get(text, text)
        text = text.replace(" ", "_")
        return text

    @staticmethod
    def _context_has_sam_prompt(vlm_context_json: str) -> bool:
        if not vlm_context_json:
            return False
        try:
            ctx = json.loads(vlm_context_json)
        except json.JSONDecodeError:
            return False
        if not isinstance(ctx, dict):
            return False
        box = ctx.get("object_box_xyxy_norm")
        return isinstance(box, list) and len(box) == 4

    def _wait_for_skill(self, future):
        deadline = time.monotonic() + self.skill_timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return None
        return future.result()

    def _wait_for_query(self, future):
        deadline = time.monotonic() + self.query_timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return None
        return future.result()

    def _build_run_skill_request(self, skill: str, request: DecodeSkill.Request):
        run_req = RunSkill.Request()
        run_req.object_name = request.object_name or ""
        run_req.target_location = request.target_location or ""
        run_req.attempt_number = int(request.attempt_number)
        if run_req.attempt_number <= 0:
            self._attempt_counter += 1
            run_req.attempt_number = self._attempt_counter

        if skill == "grasp":
            run_req.vlm_context_json = request.vlm_context_json or ""
            run_req.clear_cached_grasps = True
            run_req.force_new_perception = True
        elif skill == "pick_place":
            run_req.vlm_context_json = request.vlm_context_json or ""
            run_req.clear_cached_grasps = True
            run_req.force_new_perception = True
        elif skill == "pick":
            # Pick consume grasps ya generados; no debe regenerar SAM/GraspGen.
            run_req.vlm_context_json = ""
            run_req.clear_cached_grasps = False
            run_req.force_new_perception = False
        else:
            run_req.vlm_context_json = ""
            run_req.clear_cached_grasps = False
            run_req.force_new_perception = False

        return run_req

    def _handle_decode_skill(self, request, response):
        skill = self._normalize_skill(request.selected_skill)
        response.executed_skill = skill

        if skill not in SKILL_TO_SERVICE:
            response.success = False
            response.failure_type = "skill_not_available"
            response.message = (
                f"Skill no permitida: '{request.selected_skill}'. "
                f"Permitidas: {sorted(SKILL_TO_SERVICE.keys())}"
            )
            response.detail_json = _json_dumps({"normalized_skill": skill})
            return response

        if skill in ("grasp", "pick_place") and not self._context_has_sam_prompt(
            request.vlm_context_json
        ):
            response.success = False
            response.stage = "segmentation"
            response.failure_type = "segmentation_failed"
            response.message = (
                f"La skill '{skill}' necesita vlm_context_json con "
                "object_box_xyxy_norm para SAM2."
            )
            response.detail_json = _json_dumps({"normalized_skill": skill})
            return response

        service_name = SKILL_TO_SERVICE[skill]
        response.service_name = service_name
        client = self.skill_clients[skill]
        if not client.wait_for_service(timeout_sec=self.service_wait_timeout):
            response.success = False
            response.failure_type = "execution_failed"
            response.message = f"Servicio '{service_name}' no disponible"
            response.detail_json = _json_dumps({"normalized_skill": skill})
            return response

        run_req = self._build_run_skill_request(skill, request)
        self.get_logger().info(
            f"Decodificando skill '{request.selected_skill}' -> '{skill}' -> {service_name}"
        )
        future = client.call_async(run_req)
        skill_response = self._wait_for_skill(future)
        if skill_response is None:
            response.success = False
            response.stage = "execution"
            response.failure_type = "execution_failed"
            response.message = f"Timeout ejecutando '{service_name}'"
            response.detail_json = _json_dumps(
                {"normalized_skill": skill, "service_name": service_name}
            )
            return response

        response.success = bool(skill_response.success)
        response.stage = skill_response.stage
        response.failure_type = skill_response.failure_type
        response.message = (
            f"Skill '{skill}' ejecutada correctamente"
            if skill_response.success
            else f"Skill '{skill}' fallo"
        )
        response.detail_json = _json_dumps(
            {
                "normalized_skill": skill,
                "service_name": service_name,
                "run_skill_request": {
                    "object_name": run_req.object_name,
                    "target_location": run_req.target_location,
                    "clear_cached_grasps": run_req.clear_cached_grasps,
                    "force_new_perception": run_req.force_new_perception,
                    "has_vlm_context_json": bool(run_req.vlm_context_json),
                    "attempt_number": run_req.attempt_number,
                },
                "run_skill_response": {
                    "success": bool(skill_response.success),
                    "stage": skill_response.stage,
                    "failure_type": skill_response.failure_type,
                    "grasp_id": skill_response.grasp_id,
                    "mask_id": skill_response.mask_id,
                    "detail_json": skill_response.detail_json,
                },
            }
        )
        return response

    def _execute_decoded_request(self, decode_request: DecodeSkill.Request):
        decode_response = DecodeSkill.Response()
        return self._handle_decode_skill(decode_request, decode_response)

    @staticmethod
    def _query_response_to_json(query_response):
        return _json_dumps(
            {
                "success": bool(query_response.success),
                "selected_skill": query_response.selected_skill,
                "object_name": query_response.object_name,
                "target_location": query_response.target_location,
                "vlm_context_json": query_response.vlm_context_json,
                "message": query_response.message,
            }
        )

    def _handle_execute_instruction(self, request, response):
        instruction = (request.instruction or "").strip()
        if not instruction:
            response.success = False
            response.failure_type = "execution_failed"
            response.message = "instruction vacia"
            return response

        if not self.query_instruction_client.wait_for_service(
            timeout_sec=self.service_wait_timeout
        ):
            response.success = False
            response.failure_type = "execution_failed"
            response.message = (
                f"Servicio '{self.query_instruction_service_name}' no disponible"
            )
            return response

        query_request = QueryInstruction.Request()
        query_request.instruction = instruction
        self.get_logger().info(f"Ejecutando instruccion natural: '{instruction}'")
        query_future = self.query_instruction_client.call_async(query_request)
        query_response = self._wait_for_query(query_future)
        if query_response is None:
            response.success = False
            response.stage = "perception"
            response.failure_type = "execution_failed"
            response.message = (
                f"Timeout consultando '{self.query_instruction_service_name}'"
            )
            return response

        response.selected_skill = query_response.selected_skill
        response.object_name = query_response.object_name
        response.target_location = query_response.target_location
        response.query_response_json = self._query_response_to_json(query_response)
        response.raw_response_json = query_response.raw_response_json

        if not query_response.success:
            response.success = False
            response.stage = "perception"
            response.failure_type = "object_not_found"
            response.message = f"Query VLM fallo: {query_response.message}"
            return response

        decode_request = DecodeSkill.Request()
        decode_request.selected_skill = query_response.selected_skill
        decode_request.object_name = query_response.object_name
        decode_request.target_location = query_response.target_location
        decode_request.vlm_context_json = query_response.vlm_context_json
        self._attempt_counter += 1
        decode_request.attempt_number = self._attempt_counter

        decode_response = self._execute_decoded_request(decode_request)
        response.success = bool(decode_response.success)
        response.executed_skill = decode_response.executed_skill
        response.service_name = decode_response.service_name
        response.stage = decode_response.stage
        response.failure_type = decode_response.failure_type
        response.message = decode_response.message
        response.decode_detail_json = decode_response.detail_json
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SkillDecoderNode()
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
