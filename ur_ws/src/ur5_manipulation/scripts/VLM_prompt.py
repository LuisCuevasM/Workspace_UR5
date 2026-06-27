#!/usr/bin/env python3

import argparse
import json
import os
import re
import subprocess
import threading
from pathlib import Path
from typing import List, Literal

import requests
import requests.exceptions

import numpy as np
from PIL import Image, ImageDraw
from pydantic import BaseModel, Field, ValidationError, field_validator

try:
    import rclpy
    from rclpy.node import Node
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image as RosImage
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from ur5_manipulation.srv import SetVlmInstruction
except ImportError:
    rclpy = None
    Node = object
    CvBridge = None
    RosImage = None
    String = None
    Trigger = None
    SetVlmInstruction = None


_DEFAULT_VLM_SERVER_URL = "http://192.168.1.110:8888/infer_image"


class SemanticPartPlan(BaseModel):
    target_object: str = Field(
        description="Objeto objetivo solicitado por el usuario."
    )

    selected_safe_part: str = Field(
        description="Parte específica del objeto que se debe segmentar para agarrar."
    )

    forbidden_parts: List[str] = Field(
        description="Partes del objeto que no deben tocarse o comprimirse."
    )

    object_box_xyxy_norm: List[float] = Field(
        description=(
            "Bounding box aproximada de la parte segura seleccionada en coordenadas normalizadas "
            "[x1, y1, x2, y2], con valores entre 0 y 1."
        )
    )

    positive_points_xy_norm: List[List[float]] = Field(
        description=(
            "Puntos positivos normalizados [x, y]. Puede ser lista vacia si la caja ya es precisa."
        )
    )

    negative_points_xy_norm: List[List[float]] = Field(
        description=(
            "Puntos negativos normalizados [x, y] sobre zonas que SAM2 debe evitar."
        )
    )

    force_level: Literal["very_low", "low", "medium", "high"] = Field(
        description="Nivel de fuerza recomendado para el gripper."
    )

    confidence: Literal["low", "medium", "high"] = Field(
        description="Confianza visual de la selección."
    )

    reason: str = Field(
        description="Explicación breve de por qué esa zona es segura para agarrar."
    )

    @field_validator("object_box_xyxy_norm")
    @classmethod
    def validate_box(cls, value):
        if len(value) != 4:
            raise ValueError("object_box_xyxy_norm debe tener 4 valores: [x1, y1, x2, y2].")

        x1, y1, x2, y2 = value

        for v in value:
            if not 0.0 <= v <= 1.0:
                raise ValueError("Todos los valores de la caja deben estar entre 0 y 1.")

        if x2 <= x1 or y2 <= y1:
            raise ValueError("La caja debe cumplir x2 > x1 y y2 > y1.")

        return value

    @field_validator("positive_points_xy_norm", "negative_points_xy_norm")
    @classmethod
    def validate_points(cls, points):
        for p in points:
            if len(p) != 2:
                raise ValueError("Cada punto debe tener formato [x, y].")
            x, y = p
            if not 0.0 <= x <= 1.0 or not 0.0 <= y <= 1.0:
                raise ValueError("Cada coordenada debe estar entre 0 y 1.")
        return points


def normalize_coordinate_scale(data: dict) -> dict:
    """
    Gemma sometimes returns coordinates in a 0-1000 image scale even when asked for
    normalized 0-1 values. Convert that common format before pydantic validation.
    """
    if not isinstance(data, dict):
        raise ValueError("La respuesta JSON de Gemma debe ser un objeto.")

    coordinate_keys = (
        "object_box_xyxy_norm",
        "positive_points_xy_norm",
        "negative_points_xy_norm",
    )

    numeric_values = []
    for key in coordinate_keys:
        value = data.get(key) or []
        if key == "object_box_xyxy_norm":
            numeric_values.extend(value)
        else:
            for point in value:
                numeric_values.extend(point)

    if not numeric_values:
        return data

    max_value = max(float(value) for value in numeric_values)
    if max_value <= 1.0:
        return data

    # Gemini/Gemma vision models often use a 0-1000 coordinate convention.
    scale = 1000.0 if max_value <= 1000.0 else max_value

    normalized = dict(data)
    if normalized.get("object_box_xyxy_norm") is not None:
        normalized["object_box_xyxy_norm"] = [
            max(0.0, min(1.0, float(value) / scale))
            for value in normalized["object_box_xyxy_norm"]
        ]

    for key in ("positive_points_xy_norm", "negative_points_xy_norm"):
        if normalized.get(key) is not None:
            normalized[key] = [
                [max(0.0, min(1.0, float(x) / scale)), max(0.0, min(1.0, float(y) / scale))]
                for x, y in normalized[key]
            ]

    return normalized


def parse_response_as_plan(text: str) -> SemanticPartPlan:
    """
    Intenta parsear la respuesta como JSON.
    Si el modelo agrega texto extra, extrae el primer JSON.
    También tolera respuestas como lista de un elemento y comas finales.
    """
    if not isinstance(text, str) or not text.strip():
        raise ValueError("Gemma devolvio una respuesta vacia sin JSON.")

    def parse_json_candidate(candidate: str) -> SemanticPartPlan:
        candidate = re.sub(r",\s*([}\]])", r"\1", candidate.strip())
        data = json.loads(candidate)

        if isinstance(data, list):
            if len(data) != 1:
                raise ValueError("La respuesta JSON debe contener un único plan.")
            data = data[0]

        data = normalize_coordinate_scale(data)
        return SemanticPartPlan.model_validate(data)

    try:
        return parse_json_candidate(text)
    except (json.JSONDecodeError, TypeError, ValueError, ValidationError):
        match = re.search(r"(\{.*\}|\[.*\])", text, flags=re.DOTALL)
        if not match:
            raise
        return parse_json_candidate(match.group(0))


def _normalize_confidence(value) -> str:
    if isinstance(value, (int, float)):
        if value >= 0.75:
            return "high"
        if value >= 0.45:
            return "medium"
        return "low"
    return value


def _normalize_force_level(value) -> str:
    if isinstance(value, (int, float)):
        if value <= 0.25:
            return "very_low"
        if value <= 0.50:
            return "low"
        if value <= 0.75:
            return "medium"
        return "high"
    return value


def _select_nested_object(objects, wanted):
    """Elige el objeto de la tarea: match por nombre, si no mayor confianza visible."""
    candidates = [o for o in objects if o.get("visible", True)] or objects
    if not candidates:
        return None
    wanted = (wanted or "").strip().lower()
    if wanted:
        for o in candidates:
            if str(o.get("name", "")).lower() == wanted:
                return o
        for o in candidates:
            name = str(o.get("name", "")).lower()
            shorter, longer = sorted((name, wanted), key=len)
            if len(shorter) >= 3 and shorter in longer:
                return o

    def conf(o):
        c = o.get("confidence")
        if isinstance(c, (int, float)):
            return float(c)
        return {"low": 0.3, "medium": 0.6, "high": 0.9}.get(str(c).lower(), 0.0)

    return max(candidates, key=conf)


def flatten_nested_vlm_result(result):
    """Aplana el formato anidado del servidor VLM actual al esquema plano.

    Formato anidado:
        {"task": {"skill", "object", "target"},
         "objects": [{"name", "object_box_xyxy_norm", "confidence", "visible", ...}]}

    El formato plano (legado) pasa sin cambios.
    """
    task = result.get("task")
    if not isinstance(task, dict):
        return result

    obj = _select_nested_object(result.get("objects") or [], task.get("object")) or {}
    return {
        "target_object": str(obj.get("name") or task.get("object") or "objeto"),
        "selected_safe_part": obj.get("selected_safe_part", ""),
        "forbidden_parts": obj.get("forbidden_parts") or [],
        "object_box_xyxy_norm": obj.get("object_box_xyxy_norm") or [],
        "positive_points_xy_norm": obj.get("positive_points_xy_norm") or [],
        "negative_points_xy_norm": obj.get("negative_points_xy_norm") or [],
        "force_level": obj.get("force_level", "low"),
        "confidence": obj.get("confidence", "medium"),
        "reason": result.get("reason")
        or f"task: {task.get('skill', '')} {task.get('object', '')}".strip(),
    }


def _normalize_box_format(box):
    """Accept [[x1,y1],[x2,y2]] in addition to the canonical [x1,y1,x2,y2]."""
    if (
        isinstance(box, list)
        and len(box) == 2
        and all(isinstance(pt, (list, tuple)) and len(pt) == 2 for pt in box)
    ):
        return [box[0][0], box[0][1], box[1][0], box[1][1]]
    return box


def ask_gemma_for_part_selection(
    image_path: str,
    instruction: str,
    debug_output_path: str = None,
    vlm_server_url: str = None,
    timeout_sec: float = 60.0,
) -> SemanticPartPlan:
    url = vlm_server_url or _DEFAULT_VLM_SERVER_URL
    print(f"Consultando VLM local en {url}")
    print(f"Instruccion: {instruction}")

    try:
        with open(image_path, "rb") as img_file:
            resp = requests.post(
                url,
                files={"image": (Path(image_path).name, img_file, "image/jpeg")},
                data={"instruction": instruction},
                timeout=timeout_sec,
            )
        resp.raise_for_status()
    except requests.exceptions.ConnectionError as exc:
        raise RuntimeError(f"No se pudo conectar al servidor VLM en {url}: {exc}")
    except requests.exceptions.Timeout:
        raise RuntimeError(
            f"Timeout ({timeout_sec}s) esperando respuesta del servidor VLM {url}"
        )
    except requests.exceptions.HTTPError as exc:
        raise RuntimeError(f"Servidor VLM respondio con error HTTP: {exc}")

    try:
        response_data = resp.json()
    except Exception as exc:
        if debug_output_path:
            Path(debug_output_path).parent.mkdir(parents=True, exist_ok=True)
            Path(debug_output_path).write_bytes(resp.content)
        raise RuntimeError(
            f"Servidor VLM devolvio respuesta no JSON: {exc}\n"
            f"Contenido: {resp.content[:500]!r}"
        )

    if debug_output_path:
        debug_path = Path(debug_output_path)
        debug_path.parent.mkdir(parents=True, exist_ok=True)
        debug_path.write_text(
            json.dumps(response_data, indent=2, ensure_ascii=False), encoding="utf-8"
        )

    if not response_data.get("success"):
        error_msg = (
            response_data.get("error")
            or response_data.get("message", "Sin mensaje de error")
        )
        raise RuntimeError(f"Servidor VLM reporto error: {error_msg}")

    result = response_data.get("result")
    if result is None:
        raise RuntimeError("Servidor VLM no devolvio campo 'result' en la respuesta.")

    inference_time = response_data.get("inference_time")
    if inference_time is not None:
        print(f"VLM inference_time={inference_time:.2f}s")

    # Aplanar el formato anidado del servidor actual ({"task":..., "objects":[...]})
    result = flatten_nested_vlm_result(result)

    # Normalize alternate box format [[x1,y1],[x2,y2]] -> [x1,y1,x2,y2]
    if "object_box_xyxy_norm" in result:
        result["object_box_xyxy_norm"] = _normalize_box_format(result["object_box_xyxy_norm"])

    # Convert numeric confidence/force_level to string literals
    if "confidence" in result:
        result["confidence"] = _normalize_confidence(result["confidence"])
    if "force_level" in result:
        result["force_level"] = _normalize_force_level(result["force_level"])

    # Ensure list fields are never None
    for list_key in ("forbidden_parts", "positive_points_xy_norm", "negative_points_xy_norm"):
        if result.get(list_key) is None:
            result[list_key] = []
        elif not isinstance(result[list_key], list):
            result[list_key] = list(result[list_key])

    # Handle 0-1000 coordinate scale used by some vision models
    result = normalize_coordinate_scale(result)

    print(f"VLM target_object={result.get('target_object')}, selected_safe_part={result.get('selected_safe_part')}")

    return SemanticPartPlan.model_validate(result)


def norm_points_to_pixels(points_norm: List[List[float]], width: int, height: int) -> np.ndarray:
    points_px = []

    for x_norm, y_norm in points_norm:
        x_px = int(round(x_norm * width))
        y_px = int(round(y_norm * height))

        x_px = max(0, min(width - 1, x_px))
        y_px = max(0, min(height - 1, y_px))

        points_px.append([x_px, y_px])

    if not points_px:
        return np.empty((0, 2), dtype=np.float32)

    return np.array(points_px, dtype=np.float32)


def norm_box_to_pixels(box_norm: List[float], width: int, height: int) -> np.ndarray:
    x1, y1, x2, y2 = box_norm

    box_px = np.array(
        [
            int(round(x1 * width)),
            int(round(y1 * height)),
            int(round(x2 * width)),
            int(round(y2 * height)),
        ],
        dtype=np.float32,
    )

    box_px[0] = max(0, min(width - 1, box_px[0]))
    box_px[1] = max(0, min(height - 1, box_px[1]))
    box_px[2] = max(0, min(width - 1, box_px[2]))
    box_px[3] = max(0, min(height - 1, box_px[3]))

    return box_px


def build_sam2_prompt(plan: SemanticPartPlan, image_path: str):
    image = Image.open(image_path).convert("RGB")
    width, height = image.size

    positive_points = norm_points_to_pixels(
        plan.positive_points_xy_norm,
        width,
        height,
    )

    negative_points = norm_points_to_pixels(
        plan.negative_points_xy_norm,
        width,
        height,
    )

    if len(positive_points) > 0 and len(negative_points) > 0:
        point_coords = np.vstack([positive_points, negative_points])
    elif len(positive_points) > 0:
        point_coords = positive_points
    elif len(negative_points) > 0:
        point_coords = negative_points
    else:
        point_coords = np.empty((0, 2), dtype=np.float32)

    point_labels = np.array(
        [1] * len(positive_points) + [0] * len(negative_points),
        dtype=np.int32,
    )

    box = norm_box_to_pixels(
        plan.object_box_xyxy_norm,
        width,
        height,
    )

    return image, point_coords, point_labels, box


def save_overlay(
    image: Image.Image,
    point_coords: np.ndarray,
    point_labels: np.ndarray,
    box: np.ndarray,
    plan: SemanticPartPlan,
    output_path: str,
):
    header_height = 110
    overlay = Image.new("RGB", (image.width, image.height + header_height), "white")
    overlay.paste(image, (0, header_height))
    draw = ImageDraw.Draw(overlay)

    draw.text((12, 10), f"Objeto: {plan.target_object}", fill="black")
    draw.text((12, 30), f"Parte segura: {plan.selected_safe_part}", fill="black")
    draw.text((12, 50), f"Evitar: {', '.join(plan.forbidden_parts)}", fill="black")
    draw.text((12, 70), f"Fuerza: {plan.force_level} | Confianza: {plan.confidence}", fill="black")
    draw.text((12, 90), "Caja amarilla = parte segura | Verde = puntos positivos | Rojo = puntos negativos", fill="black")

    x1, y1, x2, y2 = box.tolist()
    draw.rectangle(
        [(x1, y1 + header_height), (x2, y2 + header_height)],
        outline="yellow",
        width=4,
    )

    radius = 8

    for (x, y), label in zip(point_coords, point_labels):
        x = int(x)
        y = int(y) + header_height

        if label == 1:
            color = "lime"
        else:
            color = "red"

        draw.ellipse(
            [(x - radius, y - radius), (x + radius, y + radius)],
            fill=color,
            outline="black",
            width=2,
        )

    overlay.save(output_path)


def save_outputs(
    plan: SemanticPartPlan,
    image: Image.Image,
    point_coords: np.ndarray,
    point_labels: np.ndarray,
    box: np.ndarray,
    output_dir: str,
):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    json_path = output_dir / "gemma_plan.json"
    overlay_path = output_dir / "gemma_visual_result.jpg"

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(plan.model_dump(), f, indent=2, ensure_ascii=False)

    save_overlay(
        image=image,
        point_coords=point_coords,
        point_labels=point_labels,
        box=box,
        plan=plan,
        output_path=str(overlay_path),
    )

    print("\nArchivos generados:")
    print(f"- JSON Gemma:       {json_path}")
    print(f"- Gráfico visual:   {overlay_path}")


def build_sam2_prompt_payload(
    plan: SemanticPartPlan,
    image: Image.Image,
    point_coords: np.ndarray,
    point_labels: np.ndarray,
    box: np.ndarray,
) -> dict:
    return {
        "target_object": plan.target_object,
        "selected_safe_part": plan.selected_safe_part,
        "forbidden_parts": plan.forbidden_parts,
        "force_level": plan.force_level,
        "confidence": plan.confidence,
        "reason": plan.reason,
        "image_width": image.width,
        "image_height": image.height,
        "object_box_xyxy_norm": plan.object_box_xyxy_norm,
        "positive_points_xy_norm": plan.positive_points_xy_norm,
        "negative_points_xy_norm": plan.negative_points_xy_norm,
        "box_xyxy": box.astype(float).tolist(),
        "point_coords": point_coords.astype(float).tolist(),
        "point_labels": point_labels.astype(int).tolist(),
    }


def force_level_to_gripper_config(plan: SemanticPartPlan) -> dict:
    mapping = {
        "very_low": {
            "close_gripper_position": 0.0,
            "close_gripper_force": 0.8,
            "close_gripper_velocity": 0.03,
        },
        "low": {
            "close_gripper_position": 0.0,
            "close_gripper_force": 1.0,
            "close_gripper_velocity": 0.04,
        },
        "medium": {
            "close_gripper_position": 0.0,
            "close_gripper_force": 1.5,
            "close_gripper_velocity": 0.05,
        },
        "high": {
            "close_gripper_position": 0.0,
            "close_gripper_force": 2.0,
            "close_gripper_velocity": 0.06,
        },
    }
    config = dict(mapping.get(plan.force_level, mapping["medium"]))

    #fragile_text = " ".join(
    #    [
    #        plan.target_object,
    #        plan.selected_safe_part,
    #        " ".join(plan.forbidden_parts),
    #        plan.reason,
    #    ]
    #).lower()
    #fragile_terms = (
    #    "lente",
    #    "lentes",
    #    "glasses",
    #    "cristal",
    #    "cristales",
    #    "vidrio",
    #    "fragil",
    #    "frágil",
    #)
    #if any(term in fragile_text for term in fragile_terms):
    #    config["close_gripper_position"] = max(
    #        config["close_gripper_position"],
    #        0.010,
    #    )

    return config



def _build_sam2_payload_from_selection(image, selection: dict) -> dict:
    width, height = image.size
    bbox = selection.get("bbox") or []
    pos_pts = selection.get("positive_points") or []
    neg_pts = selection.get("negative_points") or []

    if len(bbox) == 4:
        box_xyxy = [
            float(max(0, min(width - 1, round(bbox[0] * width)))),
            float(max(0, min(height - 1, round(bbox[1] * height)))),
            float(max(0, min(width - 1, round(bbox[2] * width)))),
            float(max(0, min(height - 1, round(bbox[3] * height)))),
        ]
    else:
        box_xyxy = []

    def _to_px(pts, w, h):
        return [
            [float(max(0, min(w - 1, round(p[0] * w)))), float(max(0, min(h - 1, round(p[1] * h))))]
            for p in pts
        ]

    pos_px = _to_px(pos_pts, width, height)
    neg_px = _to_px(neg_pts, width, height)

    conf_raw = selection.get("confidence", 0.5)
    if isinstance(conf_raw, (int, float)):
        conf_str = "high" if conf_raw >= 0.8 else ("medium" if conf_raw >= 0.5 else "low")
    else:
        conf_str = str(conf_raw)

    return {
        "target_object": selection.get("object", ""),
        "selected_safe_part": selection.get("selected_safe_part", ""),
        "forbidden_parts": selection.get("forbidden_parts") or [],
        "force_level": selection.get("force_level", "low"),
        "confidence": conf_str,
        "reason": "",
        "image_width": width,
        "image_height": height,
        "object_box_xyxy_norm": bbox,
        "positive_points_xy_norm": pos_pts,
        "negative_points_xy_norm": neg_pts,
        "box_xyxy": box_xyxy,
        "point_coords": pos_px + neg_px,
        "point_labels": [1] * len(pos_px) + [0] * len(neg_px),
    }


def _force_level_to_gripper_config_str(force_level: str) -> dict:
    mapping = {
        "very_low": {"close_gripper_position": 0.0, "close_gripper_force": 0.8, "close_gripper_velocity": 0.03},
        "low":      {"close_gripper_position": 0.0, "close_gripper_force": 1.0, "close_gripper_velocity": 0.04},
        "medium":   {"close_gripper_position": 0.0, "close_gripper_force": 1.5, "close_gripper_velocity": 0.05},
        "high":     {"close_gripper_position": 0.0, "close_gripper_force": 2.0, "close_gripper_velocity": 0.06},
    }
    return dict(mapping.get(force_level, mapping["low"]))


class VlmSam2PromptNode(Node):
    def __init__(self):
        super().__init__("vlm_sam2_prompt_node")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("prompt_topic", "/sam2/vlm_prompt")
        self.declare_parameter("gripper_config_topic", "/ur5/gripper/config")
        self.declare_parameter("select_service", "/vlm_prompt/select_object")
        self.declare_parameter("set_instruction_service", "/vlm_prompt/set_instruction")
        self.declare_parameter("sam_apply_service", "/sam2/apply_vlm_prompt")
        self.declare_parameter("instruction", "Selecciona el objeto solicitado para agarrarlo.")
        self.declare_parameter("output_dir", "/tmp/vlm_prompt")
        self.declare_parameter("save_debug_outputs", True)
        self.declare_parameter("publish_prompt_topic", False)
        # Por defecto False: en el flujo BT VLM la inferencia solo genera y
        # guarda /tmp/vlm_prompt/sam2_vlm_prompt.json. El BT recarga SAM2 y
        # llama /sam2/apply_vlm_prompt despues de liberar el VLM de la GPU.
        self.declare_parameter("call_sam_apply_service", False)
        self.declare_parameter("sam_apply_timeout_sec", 60.0)
        self.declare_parameter("vlm_server_url", _DEFAULT_VLM_SERVER_URL)  # /infer_image
        self.declare_parameter("vlm_request_timeout_sec", 60.0)
        self.declare_parameter("shared_image_filename", "latest_rgb.jpg")

        image_topic = self.get_parameter("image_topic").value
        prompt_topic = self.get_parameter("prompt_topic").value
        gripper_config_topic = self.get_parameter("gripper_config_topic").value
        select_service = self.get_parameter("select_service").value
        set_instruction_service = self.get_parameter("set_instruction_service").value
        self.current_instruction = self.get_parameter("instruction").value

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_stamp = None
        self.lock = threading.Lock()

        self.prompt_pub = self.create_publisher(String, prompt_topic, 10)
        self.gripper_config_pub = self.create_publisher(String, gripper_config_topic, 10)
        self.create_subscription(RosImage, image_topic, self._on_image, 10)
        self.create_service(Trigger, select_service, self._handle_select_object)
        self.create_service(
            SetVlmInstruction,
            set_instruction_service,
            self._handle_set_instruction,
        )

        self.declare_parameter("select_next_service", "/vlm_prompt/select_next_object")
        self.declare_parameter("reset_handled_service", "/vlm_prompt/reset_handled_objects")

        select_next_service = self.get_parameter("select_next_service").value
        reset_handled_service = self.get_parameter("reset_handled_service").value

        self.already_handled = []
        self.already_handled_lock = threading.Lock()

        self.create_service(Trigger, select_next_service, self._handle_select_next_object)
        self.create_service(Trigger, reset_handled_service, self._handle_reset_handled_objects)

        self.get_logger().info(
            f"VLM listo. imagen='{image_topic}', "
            f"prompt='{prompt_topic}', gripper='{gripper_config_topic}', "
            f"servicio='{select_service}', select_next='{select_next_service}', "
            f"instruccion='{set_instruction_service}', "
            f"servidor_vlm='{self.get_parameter('vlm_server_url').value}'."
        )

    def _handle_set_instruction(self, request, response):
        instruction = (request.instruction or "").strip()
        if not instruction:
            response.success = False
            response.message = "La instruccion no puede estar vacia."
            return response

        with self.lock:
            self.current_instruction = instruction

        response.success = True
        response.message = f"Instruccion VLM actualizada: {instruction}"
        self.get_logger().info(response.message)
        return response

    def _on_image(self, msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as exc:
            self.get_logger().warn(f"No pude convertir imagen RGB: {exc}")
            return

        with self.lock:
            self.latest_image = Image.fromarray(rgb).copy()
            self.latest_stamp = msg.header.stamp

    def _handle_select_object(self, request, response):
        del request

        self.get_logger().info("Capturando imagen actual para VLM")
        with self.lock:
            image = self.latest_image.copy() if self.latest_image is not None else None
            instruction = self.current_instruction

        if image is None:
            response.success = False
            response.message = "No hay imagen RGB disponible todavia."
            self.get_logger().error(response.message)
            return response

        output_dir = Path(self.get_parameter("output_dir").value)
        save_debug_outputs = bool(self.get_parameter("save_debug_outputs").value)
        shared_filename = str(self.get_parameter("shared_image_filename").value)
        vlm_server_url = str(self.get_parameter("vlm_server_url").value)
        vlm_timeout = float(self.get_parameter("vlm_request_timeout_sec").value)

        try:
            output_dir.mkdir(parents=True, exist_ok=True)
            image_path = output_dir / shared_filename
            image.save(str(image_path))
            self.get_logger().info(f"Imagen guardada en {image_path}")

            self.get_logger().info(
                f"Consultando VLM local en {vlm_server_url} | instruccion: '{instruction}'"
            )
            plan = ask_gemma_for_part_selection(
                str(image_path),
                instruction,
                debug_output_path=str(output_dir / "gemma_raw_response.txt"),
                vlm_server_url=vlm_server_url,
                timeout_sec=vlm_timeout,
            )
            self.get_logger().info(
                f"VLM target_object={plan.target_object}, "
                f"selected_safe_part={plan.selected_safe_part}"
            )

            image_pil, point_coords, point_labels, box = build_sam2_prompt(
                plan, str(image_path)
            )

            if save_debug_outputs:
                save_outputs(plan, image_pil, point_coords, point_labels, box, str(output_dir))

            payload = build_sam2_prompt_payload(
                plan, image_pil, point_coords, point_labels, box
            )

            # Publicar config gripper
            gripper_config = force_level_to_gripper_config(plan)
            gripper_config_msg = String()
            gripper_config_msg.data = json.dumps(gripper_config, ensure_ascii=False)
            self.gripper_config_pub.publish(gripper_config_msg)
            self.get_logger().info(
                f"Config gripper (force_level={plan.force_level}): {gripper_config_msg.data}"
            )

            # Escribir archivos de salida
            payload_path = output_dir / "sam2_vlm_prompt.json"
            with open(payload_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2, ensure_ascii=False)
            self.get_logger().info(f"Prompt SAM2 escrito en {payload_path}")

            gripper_config_path = output_dir / "gripper_config_from_vlm.json"
            with open(gripper_config_path, "w", encoding="utf-8") as f:
                json.dump(gripper_config, f, indent=2, ensure_ascii=False)

            with open(output_dir / "gemma_plan.json", "w", encoding="utf-8") as f:
                json.dump(plan.model_dump(), f, indent=2, ensure_ascii=False)

            if bool(self.get_parameter("publish_prompt_topic").value):
                msg = String()
                msg.data = json.dumps(payload, ensure_ascii=False)
                self.prompt_pub.publish(msg)

            if bool(self.get_parameter("call_sam_apply_service").value):
                sam_apply_service = self.get_parameter("sam_apply_service").value
                sam_timeout = float(self.get_parameter("sam_apply_timeout_sec").value)
                self.get_logger().info(f"Llamando {sam_apply_service}")
                result = subprocess.run(
                    [
                        "ros2", "service", "call",
                        sam_apply_service,
                        "std_srvs/srv/Trigger", "{}",
                    ],
                    check=False,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=sam_timeout,
                )
                if result.returncode != 0:
                    raise RuntimeError(
                        f"Fallo llamada a {sam_apply_service}: "
                        f"{result.stderr.strip() or result.stdout.strip()}"
                    )
                service_output = result.stdout.strip()
                if "success=False" in service_output or "success: false" in service_output.lower():
                    raise RuntimeError(
                        f"SAM2 rechazo el prompt VLM ({sam_apply_service}): {service_output}"
                    )

            response.success = True
            response.message = (
                f"VLM+SAM2 ok: objeto='{plan.target_object}', "
                f"parte='{plan.selected_safe_part}'"
            )
            self.get_logger().info(response.message)
            return response

        except Exception as exc:
            response.success = False
            response.message = f"Error en pipeline VLM/SAM2: {exc}"
            self.get_logger().error(response.message)
            return response


    def _handle_reset_handled_objects(self, request, response):
        del request
        with self.already_handled_lock:
            self.already_handled = []
        response.success = True
        response.message = "Lista de objetos manejados reiniciada."
        self.get_logger().info(response.message)
        return response

    def _handle_select_next_object(self, request, response):
        del request

        with self.lock:
            image = self.latest_image.copy() if self.latest_image is not None else None
            instruction = self.current_instruction
        with self.already_handled_lock:
            already_handled = list(self.already_handled)

        if image is None:
            response.success = False
            response.message = "No hay imagen RGB disponible todavia."
            self.get_logger().error(response.message)
            return response

        output_dir = Path(self.get_parameter("output_dir").value)
        save_debug_outputs = bool(self.get_parameter("save_debug_outputs").value)
        shared_filename = str(self.get_parameter("shared_image_filename").value)
        vlm_timeout = float(self.get_parameter("vlm_request_timeout_sec").value)
        vlm_server_url = str(self.get_parameter("vlm_server_url").value)
        base_url = vlm_server_url.rsplit("/", 1)[0]
        select_next_url = f"{base_url}/select_next_object"

        try:
            output_dir.mkdir(parents=True, exist_ok=True)
            image_path = output_dir / shared_filename
            image.save(str(image_path))
            self.get_logger().info(
                f"select_next_object -> {select_next_url} | ya_manejados={already_handled}"
            )

            with open(image_path, "rb") as img_file:
                resp = requests.post(
                    select_next_url,
                    files={"image": (shared_filename, img_file, "image/jpeg")},
                    data={
                        "instruction": instruction,
                        "already_handled": json.dumps(already_handled, ensure_ascii=False),
                    },
                    timeout=vlm_timeout,
                )
            resp.raise_for_status()
            data = resp.json()

            if save_debug_outputs:
                debug_path = output_dir / "select_next_raw_response.json"
                debug_path.write_text(
                    json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8"
                )

            if not data.get("success"):
                raise RuntimeError(data.get("error") or "select_next_object success=false")

            result = data.get("result", {})

            if result.get("done") or not result.get("visible", False):
                response.success = False
                response.message = "mesa limpia"
                self.get_logger().info("VLM select_next_object: mesa limpia (done=true o sin objeto visible).")
                return response

            payload = _build_sam2_payload_from_selection(image, result)
            payload_path = output_dir / "sam2_vlm_prompt.json"
            with open(payload_path, "w", encoding="utf-8") as pf:
                json.dump(payload, pf, indent=2, ensure_ascii=False)

            force_level = result.get("force_level", "low")
            gripper_config = _force_level_to_gripper_config_str(force_level)
            gripper_config_path = output_dir / "gripper_config_from_vlm.json"
            with open(gripper_config_path, "w", encoding="utf-8") as gf:
                json.dump(gripper_config, gf, indent=2, ensure_ascii=False)
            gripper_msg = String()
            gripper_msg.data = json.dumps(gripper_config, ensure_ascii=False)
            self.gripper_config_pub.publish(gripper_msg)

            obj_name = result.get("object", "")
            if obj_name:
                with self.already_handled_lock:
                    if obj_name not in self.already_handled:
                        self.already_handled.append(obj_name)

            response.success = True
            response.message = obj_name
            self.get_logger().info(
                f"select_next_object: objeto='{obj_name}', "
                f"parte='{result.get('selected_safe_part', '')}', "
                f"confidence={result.get('confidence', 0.0)}"
            )
            return response

        except Exception as exc:
            response.success = False
            response.message = f"Error en select_next_object: {exc}"
            self.get_logger().error(response.message)
            return response


def print_sam2_code_hint():
    print(
        """
Uso directo en SAM2:

    data = np.load("outputs/sam2_prompt.npz")

    point_coords = data["point_coords"]
    point_labels = data["point_labels"]
    box = data["box"]

    predictor.set_image(image_rgb)

    masks, scores, logits = predictor.predict(
        point_coords=point_coords,
        point_labels=point_labels,
        box=box,
        multimask_output=True,
    )

Convención:
    point_labels = 1  -> punto positivo
    point_labels = 0  -> punto negativo
"""
    )


def main():
    parser = argparse.ArgumentParser(
        description="Usa Gemma 4 para seleccionar una parte segura del objeto y visualizar caja/puntos."
    )

    parser.add_argument(
        "--ros-node",
        action="store_true",
        help="Ejecuta como nodo ROS 2: subscribe imagen, ofrece servicio y publica prompt para SAM2.",
    )

    parser.add_argument(
        "--image",
        required=False,
        help="Ruta de la imagen RGB de la mesa. Ejemplo: mesa.jpg",
    )

    parser.add_argument(
        "--instruction",
        required=False,
        help='Instrucción del usuario. Ejemplo: "Pásame los lentes"',
    )

    parser.add_argument(
        "--output-dir",
        default="outputs",
        help="Carpeta donde se guardan gemma_plan.json y gemma_visual_result.jpg.",
    )

    args, _ = parser.parse_known_args()

    if args.ros_node:
        if rclpy is None:
            raise RuntimeError("rclpy/cv_bridge no estan disponibles para modo ROS 2.")
        rclpy.init()
        node = VlmSam2PromptNode()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    if not args.image:
        parser.error("--image es requerido si no usas --ros-node.")
    if not args.instruction:
        parser.error("--instruction es requerido si no usas --ros-node.")

    if not Path(args.image).exists():
        raise FileNotFoundError(f"No existe la imagen: {args.image}")

    print("Consultando Gemma...")
    plan = ask_gemma_for_part_selection(
        image_path=args.image,
        instruction=args.instruction,
        debug_output_path=str(Path(args.output_dir) / "gemma_raw_response.txt"),
    )

    print("\nRespuesta estructurada de Gemma:")
    print(plan.model_dump_json(indent=2))

    image, point_coords, point_labels, box = build_sam2_prompt(
        plan=plan,
        image_path=args.image,
    )

    print("\nDatos visuales generados desde el plan:")
    print("point_coords:")
    print(point_coords)
    print("point_labels:")
    print(point_labels)
    print("box:")
    print(box)

    save_outputs(
        plan=plan,
        image=image,
        point_coords=point_coords,
        point_labels=point_labels,
        box=box,
        output_dir=args.output_dir,
    )

    payload = build_sam2_prompt_payload(plan, image, point_coords, point_labels, box)
    payload_path = Path(args.output_dir) / "sam2_vlm_prompt.json"
    with open(payload_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    print(f"- Prompt SAM2 JSON: {payload_path}")


if __name__ == "__main__":
    main()
