#!/usr/bin/env python3
"""Prueba end-to-end: instruccion -> VLM (tarea + objetos) -> skill ROS2.

Demuestra el flujo completo del contrato del orquestador, pero con la
inteligencia minima embebida aqui (el orquestador LangGraph real vive en otro
repo). El servidor VLM responde con el formato anidado:

    {"result": {"task":    {"skill": "pick_place", "object": "barco", "target": "caja"},
                "objects": [{"name": ..., "object_box_xyxy_norm": [...],
                             "positive_points_xy_norm": [...], "visible": true, ...}]},
     "success": true}

(se mantiene compatibilidad con el formato plano anterior, con skill/
target_object/target_location al nivel raiz del result).

El driver ejecuta:

    /robot/get_camera_image  -> RGB actual (base64)
        -> VLM /infer_image  -> task {skill, object, target} + objects []
    /robot/perceive          -> mask_id + pose + nube segmentada por objeto
    /robot/<skill>           -> ejecuta la skill (p.ej. pick_place) con vlm_context_json

Si la skill falla, aplica el INVARIANTE del contrato: reintenta con
clear_cached_grasps=true y force_new_perception=true (para grasp/pick/pick_place),
hasta --max-attempts.

Modo SIN VLM (--no-vlm): salta camara/VLM/perceive y llama directo a
/robot/<skill> con vlm_context_json vacio; robot_skill_node entonces usa el
ciclo de preparacion sin VLM (/ur5/prepare_bt_grasp_cycle, seleccion SAM2).

Transporte: rosbridge websocket (roslibpy), igual que el orquestador real.

Requisitos en la maquina del robot:
    pip install roslibpy requests
    ros2 launch ur5_manipulation robot_services.launch.py   # servicios + rosbridge
    (stack de bajo nivel arriba: SAM2, GraspGen, MoveIt, UR5, RealSense, VLM)

Uso:
    python3 e2e_grasp_test.py --instruction "agarra delicadamente el pato"
    python3 e2e_grasp_test.py --instruction "pon el barco en la caja"
    python3 e2e_grasp_test.py --instruction "..." --skill pick   # forzar skill
    python3 e2e_grasp_test.py --no-vlm --skill pick --object pato
    python3 e2e_grasp_test.py --no-vlm --skill pick_place --object pato --target caja
"""

import argparse
import base64
import json
import sys
import time

try:
    import roslibpy
except ImportError:
    print("ERROR: falta roslibpy. Instala con: pip install roslibpy", file=sys.stderr)
    raise

try:
    import requests
except ImportError:
    print("ERROR: falta requests. Instala con: pip install requests", file=sys.stderr)
    raise


# Tipos de servicio (convencion rosbridge ROS2: '<pkg>/srv/<Tipo>').
SRV_GET_IMAGE = "ur5_manipulation/srv/GetCameraImage"
SRV_PERCEIVE = "ur5_manipulation/srv/Perceive"
SRV_RUN_SKILL = "ur5_manipulation/srv/RunSkill"

IMPLEMENTED_SKILLS = {"grasp", "pick", "place", "pick_place"}
ALL_SKILLS = IMPLEMENTED_SKILLS | {"push", "handover"}
SKILLS_WITH_REPLAN_INVARIANT = {"grasp", "pick", "pick_place"}


def log(section, msg):
    print(f"[{section}] {msg}", flush=True)


def banner(title):
    print("\n" + "=" * 70)
    print(f" {title}")
    print("=" * 70, flush=True)


def call_service(client, name, srv_type, request, timeout):
    """Llamada sincrona a un servicio ROS2 via rosbridge."""
    service = roslibpy.Service(client, name, srv_type)
    req = roslibpy.ServiceRequest(request)
    log("ros", f"-> {name} {json.dumps(request)[:160]}")
    result = service.call(req, timeout=timeout)
    return result


def query_vlm(vlm_url, image_bytes, instruction, timeout):
    """POST de la imagen + instruccion al VLM Gemma -> dict con las decisiones."""
    files = {"image": ("frame.jpg", image_bytes, "image/jpeg")}
    data = {"instruction": instruction}
    log("vlm", f"POST {vlm_url}  instruction='{instruction}'")
    resp = requests.post(vlm_url, files=files, data=data, timeout=timeout)
    resp.raise_for_status()
    payload = resp.json()
    if not payload.get("success", False):
        raise RuntimeError(f"VLM fallo: {payload}")
    result = payload.get("result")
    if not isinstance(result, dict):
        raise RuntimeError(f"VLM no devolvio JSON estructurado: {payload}")
    return result


def _normalized_object(obj):
    """Esquema por-objeto que consume /robot/perceive (campos extra se ignoran)."""
    return {
        "name": str(obj.get("name") or obj.get("target_object") or "objeto"),
        "object_box_xyxy_norm": obj.get("object_box_xyxy_norm") or [],
        "positive_points_xy_norm": obj.get("positive_points_xy_norm") or [],
        "negative_points_xy_norm": obj.get("negative_points_xy_norm") or [],
        "selected_safe_part": obj.get("selected_safe_part", ""),
        "forbidden_parts": obj.get("forbidden_parts") or [],
        "force_level": obj.get("force_level", "low"),
        "confidence": obj.get("confidence"),
        "visible": bool(obj.get("visible", True)),
    }


def normalize_vlm_result(result):
    """Adapta la respuesta del VLM a (task, objects).

    Formato actual del servidor (anidado):
        {"task": {"skill", "object", "target"}, "objects": [{...}, ...]}
    Formato plano legado (un solo objeto al nivel raiz):
        {"skill", "target_object", "target_location", "object_box_xyxy_norm", ...}
    """
    if isinstance(result.get("task"), dict):
        task = result["task"]
        objects = [
            _normalized_object(o)
            for o in (result.get("objects") or [])
            if isinstance(o, dict)
        ]
        return {
            "skill": task.get("skill", ""),
            "object": task.get("object", ""),
            "target": task.get("target", "") or "",
            "reason": result.get("reason", ""),
        }, objects

    return {
        "skill": result.get("skill", ""),
        "object": result.get("target_object", ""),
        "target": result.get("target_location", "") or "",
        "reason": result.get("reason", ""),
    }, [_normalized_object(result)]


def select_target_object(objects, task_object):
    """Elige el objeto detectado que corresponde al objetivo de la tarea.

    El VLM puede nombrar el objeto de la tarea en otro idioma que la deteccion
    (p.ej. task.object="barco", objects[0].name="small blue plastic item"), asi
    que el match por nombre es best-effort y el fallback es la mayor confianza.
    """
    candidates = [o for o in objects if o["visible"]] or objects
    if not candidates:
        return None
    wanted = (task_object or "").strip().lower()
    if wanted:
        for o in candidates:
            if o["name"].lower() == wanted:
                return o
        for o in candidates:
            name = o["name"].lower()
            shorter, longer = sorted((name, wanted), key=len)
            if len(shorter) >= 3 and shorter in longer:
                return o

    def conf(o):
        c = o.get("confidence")
        if isinstance(c, (int, float)):
            return float(c)
        return {"low": 0.3, "medium": 0.6, "high": 0.9}.get(str(c).lower(), 0.0)

    return max(candidates, key=conf)


def run_skill_once(client, skill, object_name, target_location, vlm_context,
                   clear_cached, force_new, attempt, timeout):
    request = {
        "object_name": object_name,
        "target_location": target_location,
        "clear_cached_grasps": clear_cached,
        "force_new_perception": force_new,
        # Vacio (modo --no-vlm) => robot_skill_node usa el prepare sin VLM.
        "vlm_context_json": json.dumps(vlm_context) if vlm_context else "",
        "attempt_number": attempt,
    }
    return call_service(client, f"/robot/{skill}", SRV_RUN_SKILL, request, timeout)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instruction", default="",
                        help='p.ej. "agarra delicadamente el pato" (requerido salvo --no-vlm)')
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--vlm-url", default="http://172.17.0.2:8888/infer_image")
    parser.add_argument("--no-vlm", action="store_true",
                        help="no consultar el VLM: llamar /robot/<skill> directo "
                             "(usa el ciclo de preparacion sin VLM)")
    parser.add_argument("--skill", default="", choices=[""] + sorted(ALL_SKILLS),
                        help="forzar la skill en vez de usar la decision del VLM "
                             "(requerido con --no-vlm)")
    parser.add_argument("--object", default="",
                        help="forzar el nombre del objeto (util con --no-vlm)")
    parser.add_argument("--target", default="",
                        help="target_location para place/pick_place (forzado)")
    parser.add_argument("--max-attempts", type=int, default=3)
    parser.add_argument("--service-timeout", type=float, default=120.0)
    parser.add_argument("--vlm-timeout", type=float, default=180.0)
    args = parser.parse_args()

    if args.no_vlm and not args.skill:
        parser.error("--no-vlm requiere --skill")
    if not args.no_vlm and not args.instruction:
        parser.error("--instruction es requerido salvo en modo --no-vlm")

    headline = args.instruction or f"(--no-vlm) skill={args.skill}"
    banner(f'INSTRUCCION: "{headline}"')

    client = roslibpy.Ros(host=args.host, port=args.port)
    client.run()
    if not client.is_connected:
        log("ros", f"No se pudo conectar a rosbridge ws://{args.host}:{args.port}")
        return 2
    log("ros", f"Conectado a rosbridge ws://{args.host}:{args.port}")

    try:
        vlm_context = None

        if args.no_vlm:
            skill = args.skill
            object_name = args.object or "objeto"
            target_location = args.target
            banner("MODO SIN VLM (skill directa)")
            log("decision", f"skill = {skill} (CLI)  objeto = '{object_name}'  "
                            f"target_location = '{target_location}'")
        else:
            # 1) Imagen actual de la camara.
            img = call_service(client, "/robot/get_camera_image", SRV_GET_IMAGE, {},
                               args.service_timeout)
            if not img.get("success"):
                log("img", f"get_camera_image fallo: {img.get('detail')}")
                return 1
            image_bytes = base64.b64decode(img["image_base64"])
            log("img", f"RGB {img['width']}x{img['height']} frame='{img['frame_id']}' "
                       f"stamp={img['stamp']:.3f}")

            # 2) VLM: tarea (skill/objeto/destino) + objetos detectados.
            vlm = query_vlm(args.vlm_url, image_bytes, args.instruction,
                            args.vlm_timeout)
            task, objects = normalize_vlm_result(vlm)
            skill = args.skill or task["skill"] or "pick"
            object_name = args.object or task["object"] or "objeto"
            target_location = args.target or task["target"]

            target_obj = select_target_object(objects, task["object"])
            if target_obj is None or len(target_obj["object_box_xyxy_norm"]) != 4:
                log("guard", "el VLM no devolvio objetos con box valido; abortando.")
                return 1

            banner("DECISIONES DEL ROBOT (VLM)")
            log("decision-1", f"skill        = {skill}"
                              + (" (forzada por --skill)" if args.skill else " (VLM)"))
            log("decision-2", f"force_level  = {target_obj['force_level']}  (delicadeza)")
            log("vlm", f"task: object='{task['object']}' target='{task['target']}' | "
                       f"{len(objects)} objeto(s) detectado(s)")
            log("vlm", f"objetivo = '{target_obj['name']}' "
                       f"confidence={target_obj.get('confidence')} "
                       f"safe_part='{target_obj['selected_safe_part']}'")
            if task["reason"]:
                log("vlm", f"reason = {task['reason']}")

        # Guarda determinista (como hace select_skill del orquestador).
        if skill not in ALL_SKILLS:
            log("guard", f"skill '{skill}' desconocida; abortando.")
            return 1
        if skill not in IMPLEMENTED_SKILLS:
            log("guard", f"skill '{skill}' declarada pero NO implementada; "
                         "se llamara igual y debe responder skill_not_available.")
        if skill in ("place", "pick_place") and not target_location:
            log("guard", f"skill '{skill}' requiere target_location y vino vacio; abortando.")
            return 1

        if not args.no_vlm:
            # 3) Percepcion: SAM2 + depth -> mask_id, pose, nube (todos los
            #    objetos visibles con box; el de la tarea se usa para la skill).
            perceive_objs = [
                o for o in objects
                if o["visible"] and len(o["object_box_xyxy_norm"]) == 4
            ]
            perceive_resp = call_service(
                client, "/robot/perceive", SRV_PERCEIVE,
                {"stamp": img["stamp"], "objects_json": json.dumps(perceive_objs)},
                args.service_timeout,
            )
            enriched = {}
            if perceive_resp.get("success"):
                objs = json.loads(perceive_resp.get("objects_json", "[]"))
                enriched = next(
                    (o for o in objs if o.get("name") == target_obj["name"]),
                    objs[0] if objs else {},
                )
                for o in objs:
                    log("perceive", f"'{o.get('name')}': ok={o.get('segmentation_ok')} "
                                    f"mask_id={o.get('mask_id')} "
                                    f"pose={o.get('pose_estimate')} "
                                    f"cloud={o.get('cloud_topic')} "
                                    f"pts={o.get('num_cloud_points')}")
            else:
                log("perceive", f"perceive fallo: {perceive_resp.get('failure_type')} "
                                f"{perceive_resp.get('detail')} (continuo; la skill puede "
                                "re-percibir con force_new_perception).")

            # 4) Contexto VLM serializado para la skill (incluye mask_id + pose).
            vlm_context = dict(target_obj)
            vlm_context["task"] = dict(task)
            vlm_context["mask_id"] = enriched.get("mask_id", "")
            vlm_context["pose_estimate"] = enriched.get("pose_estimate", [])

        # 5) Ejecutar la skill, con el invariante de replan ante fallo.
        banner(f"EJECUTANDO SKILL: /robot/{skill}")
        clear_cached, force_new = False, False
        final = None
        for attempt in range(1, args.max_attempts + 1):
            log("skill", f"intento {attempt}/{args.max_attempts} "
                         f"(clear_cached_grasps={clear_cached}, "
                         f"force_new_perception={force_new})")
            final = run_skill_once(
                client, skill, object_name, target_location, vlm_context,
                clear_cached, force_new, attempt, args.service_timeout,
            )
            ok = final.get("success", False)
            log("skill", f"-> success={ok} stage={final.get('stage')} "
                         f"failure_type='{final.get('failure_type')}' "
                         f"grasp_id={final.get('grasp_id')} mask_id={final.get('mask_id')}")
            if ok:
                break
            if skill not in IMPLEMENTED_SKILLS:
                break  # push/handover: skill_not_available, no tiene sentido reintentar
            # Invariante del contrato: tras un fallo de grasp/pick/pick_place,
            # SIEMPRE reintentar sin reutilizar agarres ni mascaras. Sin VLM no
            # hay prompt que re-publicar: el prepare no-VLM re-segmenta solo.
            if skill in SKILLS_WITH_REPLAN_INVARIANT:
                clear_cached, force_new = True, vlm_context is not None

        banner("RESULTADO FINAL")
        if final and final.get("success"):
            log("done", f"OK: '{args.instruction}' -> skill '{skill}' completada.")
            rc = 0
        else:
            ft = final.get("failure_type") if final else "sin_respuesta"
            log("done", f"FALLO: failure_type='{ft}' stage='{final.get('stage') if final else ''}'")
            detail = final.get("detail_json") if final else ""
            if detail:
                log("done", f"detail_json={detail}")
            rc = 1
        return rc

    finally:
        client.terminate()


if __name__ == "__main__":
    sys.exit(main())
