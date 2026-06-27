#!/usr/bin/env python3

import json
import threading

import cv2
import requests
import requests.exceptions

import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from ur5_manipulation.srv import QueryInstruction


ALLOWED_SKILLS = {
    "clean_table",
    "grasp",
    "pick",
    "place",
    "pick_place",
    "push",
    "handover",
}

SKILL_ALIASES = {
    "clean table": "clean_table",
    "clean_the_table": "clean_table",
    "limpiar_mesa": "clean_table",
    "limpia_mesa": "clean_table",
    "limpiar_la_mesa": "clean_table",
    "limpia_la_mesa": "clean_table",
    "pick place": "pick_place",
    "pick_and_place": "pick_place",
    "pickplace": "pick_place",
}


def _json_dumps(data) -> str:
    return json.dumps(data, ensure_ascii=False, separators=(",", ":"))


def _normalize_skill(value) -> str:
    skill = str(value or "").strip().lower().replace("-", "_")
    skill = " ".join(skill.split())
    skill = SKILL_ALIASES.get(skill, skill)
    skill = skill.replace(" ", "_")
    return SKILL_ALIASES.get(skill, skill)


def _unwrap_vlm_response(response_data, endpoint):
    if not isinstance(response_data, dict):
        raise ValueError(f"{endpoint} no devolvio JSON object.")

    if not response_data.get("success"):
        raise ValueError(response_data.get("error") or f"{endpoint} success=false")

    result = response_data.get("result")
    if not isinstance(result, dict):
        raise ValueError(f"{endpoint} no contiene result como objeto.")
    return result


def _selector_to_vlm_context(selection):
    return {
        "name": selection.get("object", ""),
        "visible": selection.get("visible", False),
        "object_box_xyxy_norm": selection.get("bbox", []),
        "positive_points_xy_norm": selection.get("positive_points", []),
        "negative_points_xy_norm": selection.get("negative_points", []),
        "selected_safe_part": selection.get("selected_safe_part") or "",
        "forbidden_parts": selection.get("forbidden_parts", []),
        "force_level": selection.get("force_level", "low"),
        "confidence": selection.get("confidence", 0.0),
    }


class VlmInstructionQueryNode(Node):
    def __init__(self):
        super().__init__("vlm_instruction_query_node")

        self.cb_group = ReentrantCallbackGroup()
        self.declare_parameter("service_name", "/ur5/query_instruction")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("vlm_base_url", "http://192.168.1.110:8888")
        self.declare_parameter("vlm_request_timeout_sec", 180.0)
        self.declare_parameter("jpeg_quality", 95)

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_jpeg = None

        image_topic = self.get_parameter("image_topic").value
        service_name = self.get_parameter("service_name").value

        self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            10,
            callback_group=self.cb_group,
        )
        self.create_service(
            QueryInstruction,
            service_name,
            self._handle_query_instruction,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            "vlm_instruction_query_node listo: "
            f"service='{service_name}', image_topic='{image_topic}', "
            f"vlm_base='{self.get_parameter('vlm_base_url').value}'."
        )

    def _on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            quality = int(self.get_parameter("jpeg_quality").value)
            ok, buffer = cv2.imencode(
                ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            )
            if not ok:
                raise RuntimeError("cv2.imencode JPEG fallo")
        except Exception as exc:
            self.get_logger().warn(f"No pude preparar imagen RGB para VLM: {exc}")
            return

        with self.lock:
            self.latest_jpeg = bytes(buffer)


    def _handle_query_instruction(self, request, response):
        instruction = (request.instruction or "").strip()
        if not instruction:
            response.success = False
            response.message = "La instruccion no puede estar vacia."
            return response

        vlm_base_url = str(self.get_parameter("vlm_base_url").value).rstrip("/")
        timeout = float(self.get_parameter("vlm_request_timeout_sec").value)
        response_data = None

        try:
            interpret_url = f"{vlm_base_url}/interpret_skill"
            self.get_logger().info(
                f"Consultando Skill Router en {interpret_url}: '{instruction}'"
            )
            interpret_response = requests.post(
                interpret_url,
                json={"instruction": instruction},
                timeout=timeout,
            )
            interpret_response.raise_for_status()
            interpret_data = interpret_response.json()
            task = _unwrap_vlm_response(interpret_data, "/interpret_skill")

            skill = _normalize_skill(task.get("skill"))
            if skill not in ALLOWED_SKILLS:
                raise ValueError(
                    f"Skill VLM no permitida: '{skill}'. Permitidas: {sorted(ALLOWED_SKILLS)}"
                )

            object_name = str(task.get("object") or "").strip()
            target_location = str(task.get("target") or "").strip()
            needs_object_selection = bool(task.get("needs_object_selection", True))

            vlm_context = {}
            response_data = {"interpret_skill": interpret_data}

            if needs_object_selection:
                if not object_name:
                    raise ValueError(
                        f"Skill '{skill}' requiere object pero el router no lo devolvio."
                    )

                with self.lock:
                    image_bytes = self.latest_jpeg
                if image_bytes is None:
                    response.success = False
                    response.message = "No hay imagen RGB disponible todavia."
                    return response

                select_url = f"{vlm_base_url}/select_object"
                self.get_logger().info(
                    f"Consultando Object Selector en {select_url}: object='{object_name}'"
                )
                select_response = requests.post(
                    select_url,
                    files={"image": ("query_instruction.jpg", image_bytes, "image/jpeg")},
                    data={
                        "object": object_name,
                        "instruction": instruction,
                        "target": target_location,
                    },
                    timeout=timeout,
                )
                select_response.raise_for_status()
                select_data = select_response.json()
                selection = _unwrap_vlm_response(select_data, "/select_object")
                response_data["select_object"] = select_data

                if not selection.get("visible", False):
                    raise ValueError(f"Objeto '{object_name}' no visible para el VLM.")
                if float(selection.get("confidence", 0.0)) <= 0.0:
                    raise ValueError(f"Objeto '{object_name}' con confianza 0.0.")

                vlm_context = _selector_to_vlm_context(selection)

            response.raw_response_json = _json_dumps(response_data)
            response.success = True
            response.selected_skill = skill
            response.object_name = object_name
            response.target_location = target_location
            response.vlm_context_json = _json_dumps(vlm_context) if vlm_context else ""
            response.message = f"VLM selecciono skill '{skill}'."
            return response
        except requests.exceptions.Timeout:
            response.success = False
            response.message = f"Timeout consultando VLM en {vlm_base_url}"
            return response
        except requests.exceptions.ConnectionError as exc:
            response.success = False
            response.message = f"No se pudo conectar al VLM en {vlm_base_url}: {exc}"
            return response
        except requests.exceptions.HTTPError as exc:
            response.success = False
            response.message = f"VLM respondio con error HTTP: {exc}"
            return response
        except json.JSONDecodeError as exc:
            response.success = False
            response.message = f"VLM devolvio respuesta no JSON: {exc}"
            return response
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            if response_data is not None and not response.raw_response_json:
                response.raw_response_json = _json_dumps(response_data)
            return response
        except Exception as exc:
            response.success = False
            response.message = f"Fallo inesperado consultando VLM: {exc}"
            return response


def main(args=None):
    rclpy.init(args=args)
    node = VlmInstructionQueryNode()
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
