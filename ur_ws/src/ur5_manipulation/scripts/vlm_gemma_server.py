#!/usr/bin/env python3
"""
Servidor Flask para inferencia VLM local con Gemma.

Endpoints:
  GET  /health         — estado del servidor y de la GPU.
  POST /infer          — JSON body: {"image_path": "...", "instruction": "..."}
  POST /infer_image    — multipart: image=<file>, instruction=<string>
  POST /release        — libera el modelo de la GPU (sin apagar el servidor).

Arquitectura:
  - El servidor Flask permanece vivo todo el tiempo.
  - El modelo se carga BAJO DEMANDA (init_model) en la primera inferencia.
  - Tras inferir, el cliente (BT) llama /release para liberar la VRAM del
    modelo de modo que SAM2/GraspGen puedan usar la GPU.

Copia este archivo al Docker Gemma e inícialo con:
  python3 vlm_gemma_server.py
"""

from __future__ import annotations

import os
# expandable_segments: reduce fragmentacion en cargas/descargas repetidas.
# PYTORCH_NO_CUDA_MEMORY_CACHING=1 hace que PyTorch llame cudaFree() de
# inmediato al borrar tensores, en lugar de guardarlos en cache. Esto permite
# que el driver recupere paginas GPU mas rapido tras /release, lo cual es
# critico cuando SAM2 y el VLM comparten la misma GPU.
os.environ.setdefault(
    "PYTORCH_CUDA_ALLOC_CONF",
    "expandable_segments:True,garbage_collection_threshold:0.8",
)
os.environ.setdefault("PYTORCH_NO_CUDA_MEMORY_CACHING", "1")

import json
import re
import time
import traceback
import gc

from flask import Flask, request, jsonify
from PIL import Image

import torch
from transformers import (
    AutoProcessor,
    AutoModelForImageTextToText,
)

app = Flask(__name__)

MODEL_ID = "google/gemma-4-E2B-it"

processor = None
model = None


SYSTEM_PROMPT = """
You are a robotics vision-language advisor for robotic grasping.

Analyze the image and the user instruction.

Return ONLY valid JSON inside one JSON object.

Required fields:
- skill: one of "grasp", "pick", "place", "pick_place", "push", "handover"
- target_object: string
- target_location: string (where to place the object; "" if not applicable)
- selected_safe_part: string
- forbidden_parts: list of strings
- object_box_xyxy_norm: list [x1, y1, x2, y2], normalized between 0 and 1
- positive_points_xy_norm: list of [x, y] normalized points
- negative_points_xy_norm: list of [x, y] normalized points
- force_level: one of "very_low", "low", "medium", "high"
- confidence: one of "low", "medium", "high"
- reason: string

Skill decision (choose exactly one for the "skill" field):
- "pick": user wants to grab/take/lift an object and hold it
  (e.g. "agarra el pato", "toma la taza", "levanta la caja", "pick up the duck").
  This is the default for a grasp-and-hold request.
- "grasp": user wants only to close the gripper on the object WITHOUT lifting it.
- "pick_place": user wants to move an object to a destination
  (e.g. "pon el pato en la caja", "coloca la taza sobre la mesa", "mete X en Y").
  Set target_location to the destination (e.g. "caja").
- "place": user wants to release/deposit an object already held by the gripper.
- "push": user wants to push/slide an object without grasping.
- "handover": user wants the robot to hand the object to a person.
- target_location must be "" unless skill is "place" or "pick_place".

Rules:
- Coordinates must be normalized between 0 and 1.
- Do not use pixel coordinates.
- Do not use 0-1000 coordinates.
- object_box_xyxy_norm must be [x1, y1, x2, y2], not [[x1,y1],[x2,y2]].
- Select the safest visible graspable part, not necessarily the full object.
- Never select the table, background, shadows, robot, hand, or nearby objects.
- If the user asks for glasses or lentes:
  - target_object must be "glasses".
  - selected_safe_part must be "left_temple" or "right_temple".
  - forbidden_parts must include "lenses".
  - Avoid selecting the lenses or center frame.
  - Use force_level "very_low" or "low".
- positive_points_xy_norm may be empty if the bounding box is precise.
- negative_points_xy_norm should include fragile parts or confusing nearby regions.
- Return JSON only. No markdown. No explanation outside the JSON.
"""


def extract_json(text: str):
    text = text.strip()

    code_block_matches = re.findall(
        r"```json\s*(\{.*?\})\s*```",
        text,
        flags=re.DOTALL,
    )

    if code_block_matches:
        return json.loads(code_block_matches[-1])

    text = text.replace("```json", "").replace("```", "")

    marker = text.rfind("model")
    if marker != -1:
        text = text[marker:]

    start = text.find("{")
    end = text.rfind("}")

    if start == -1 or end == -1 or end <= start:
        raise RuntimeError(f"Could not find JSON in response:\n{text}")

    return json.loads(text[start:end + 1])


def gpu_memory_info():
    if not torch.cuda.is_available():
        return {
            "cuda_allocated_gb": 0.0,
            "cuda_reserved_gb": 0.0,
        }

    return {
        "cuda_allocated_gb": torch.cuda.memory_allocated() / 1024 ** 3,
        "cuda_reserved_gb": torch.cuda.memory_reserved() / 1024 ** 3,
    }


def init_model():
    global processor
    global model

    if model is not None:
        return

    print(f"Loading {MODEL_ID}")

    processor = AutoProcessor.from_pretrained(MODEL_ID)

    model = AutoModelForImageTextToText.from_pretrained(
        MODEL_ID,
        torch_dtype=torch.bfloat16,  # "auto" puede cargar float32 (~10 GB); bfloat16 usa ~4-5 GB
        device_map="auto",
    )

    model.eval()

    print("Gemma ready")


def free_gpu_model():
    """Libera de la GPU el modelo VLM, sus hooks de accelerate y la cache.

    Con device_map="auto", accelerate instala hooks (AlignDevicesHook) en cada
    submodulo que MANTIENEN referencias a los pesos en GPU. Si solo se hace
    `del model`, esos hooks impiden que el refcount llegue a 0 y la VRAM NO se
    libera. Hay que quitar los hooks antes de soltar el modelo.

    Devuelve (allocated_before_gb, allocated_after_gb).
    """
    global model
    global processor

    before = gpu_memory_info()["cuda_allocated_gb"]

    if model is not None:
        # Quitar hooks de accelerate: con device_map="auto", accelerate instala
        # AlignDevicesHook en cada submódulo que mantiene referencias a tensores
        # en GPU. Sin esto, del model no baja el refcount a 0.
        try:
            from accelerate.hooks import remove_hook_from_module
            remove_hook_from_module(model, recurse=True)
        except Exception:
            traceback.print_exc()

        # Liberar atributos conocidos que pueden retener tensores.
        for attr in ("generation_config", "_cache", "past_key_values"):
            try:
                setattr(model, attr, None)
            except Exception:
                pass

        # Mover todos los parametros a CPU antes de soltar para forzar la
        # devolucion de paginas GPU al driver (util cuando CUDA tiene bloques
        # "reservados" que no se liberan con empty_cache en algunos drivers).
        try:
            model.cpu()
        except Exception:
            pass

    model = None
    processor = None

    # Dos pasadas de GC para romper referencias circulares que la primera
    # pasada deja en la lista de finalizacion.
    gc.collect()
    gc.collect()

    if torch.cuda.is_available():
        torch.cuda.empty_cache()
        torch.cuda.ipc_collect()
        torch.cuda.synchronize()

    after = gpu_memory_info()["cuda_allocated_gb"]
    return before, after


def run_inference(image: Image.Image, instruction: str):
    if model is None or processor is None:
        init_model()
    prompt = f""" {SYSTEM_PROMPT}
User instruction:

{instruction}
"""

    messages = [
        {
            "role": "user",
            "content": [
                {
                    "type": "image",
                    "image": image,
                },
                {
                    "type": "text",
                    "text": prompt,
                },
            ],
        }
    ]

    inputs = processor.apply_chat_template(
        messages,
        add_generation_prompt=True,
        tokenize=True,
        return_dict=True,
        return_tensors="pt",
    ).to(model.device)

    t0 = time.time()

    with torch.no_grad():
        outputs = model.generate(
            **inputs,
            max_new_tokens=512,
            do_sample=False,
            temperature=None,
            top_p=None,
        )

    inference_time = time.time() - t0

    raw_response = processor.decode(
        outputs[0],
        skip_special_tokens=True,
    )

    print("\n========== RAW RESPONSE ==========")
    print(raw_response)
    print("==================================\n")

    parsed_json = extract_json(raw_response)

    # Liberar tensores de esta inferencia para no dejar VRAM reservada de mas.
    del inputs
    del outputs

    return {
        "success": True,
        "result": parsed_json,
        "raw_response": raw_response,
        "inference_time": inference_time,
    }


@app.route("/health", methods=["GET"])
def health():
    return jsonify({
        "success": True,
        "model_loaded": model is not None,
        "model": MODEL_ID,
        "cuda": torch.cuda.is_available(),
        "gpu": (
            torch.cuda.get_device_name(0)
            if torch.cuda.is_available()
            else None
        ),
        **gpu_memory_info(),
    })


@app.route("/infer", methods=["POST"])
def infer():
    try:
        data = request.get_json(force=True)

        image_path = data.get("image_path")
        instruction = data.get("instruction")

        if image_path is None:
            return jsonify({
                "success": False,
                "error": "Missing image_path",
            }), 400

        if instruction is None:
            return jsonify({
                "success": False,
                "error": "Missing instruction",
            }), 400

        image = Image.open(image_path).convert("RGB")
        result = run_inference(image, instruction)

        return jsonify(result)

    except Exception as e:
        traceback.print_exc()
        return jsonify({
            "success": False,
            "error": str(e),
        }), 500


@app.route("/release", methods=["POST"])
def release():
    try:
        before, after = free_gpu_model()
        print(
            f"[release] cuda_allocated_gb: {before:.3f} -> {after:.3f}",
            flush=True,
        )
        return jsonify({
            "success": True,
            "message": "VLM model released",
            "model_loaded": False,
            "cuda_allocated_before_gb": before,
            **gpu_memory_info(),
        })

    except Exception as e:
        traceback.print_exc()

        return jsonify({
            "success": False,
            "error": str(e),
        }), 500


@app.route("/infer_image", methods=["POST"])
def infer_image():
    try:
        if "image" not in request.files:
            return jsonify({
                "success": False,
                "error": "Missing multipart file field 'image'",
            }), 400

        instruction = request.form.get("instruction")

        if not instruction:
            return jsonify({
                "success": False,
                "error": "Missing form field 'instruction'",
            }), 400

        image_file = request.files["image"]
        image = Image.open(image_file.stream).convert("RGB")

        result = run_inference(image, instruction)

        return jsonify(result)

    except Exception as e:
        traceback.print_exc()
        return jsonify({
            "success": False,
            "error": str(e),
        }), 500


if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=8888,
        debug=False,
    )
