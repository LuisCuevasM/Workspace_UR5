# Plan: Arquitectura modular de manipulación robótica (VLM + LangGraph + ROS2)

## Contexto

El workspace actual contiene solo el servidor VLM ([server.py](/workspace/server.py)): Flask + Gemma (`google/gemma-4-E2B-it`) que recibe imagen + instrucción y devuelve JSON con bounding box normalizado, puntos prompt para SAM2, partes prohibidas, `force_level` y `confidence`. El stack ROS2 (SAM2, GraspGen, MoveIt2, UR5, Robotiq Hand-E) vive en otro repo/máquina.

Objetivo: que el VLM cumpla **solo percepción semántica**, que **LangGraph** haga la planificación de alto nivel (selección de skill, memoria, replanificación) y que la ejecución física quede encapsulada en **skills ROS2** accesibles vía **rosbridge**. Decisiones confirmadas con el usuario:

1. Las skills ROS2 existen/se implementan en otro repo → aquí va el servidor VLM, el orquestador LangGraph y el **contrato de interfaces** hacia ROS2.
2. Los nodos de planificación (`parse_instruction`, `select_skill`, `replan`) usan **el mismo Gemma vía HTTP** (modo texto), con guardas deterministas.
3. LangGraph ↔ ROS2 vía **rosbridge websocket** (`roslibpy`).
4. El VLM entrega solo box/puntos; un servicio ROS2 de percepción (`/robot/perceive`) lo enriquece con SAM2 + depth para producir `mask_id`, nube de puntos y `pose_estimate`.

## Arquitectura

```
Usuario (instrucción)
  └─ orchestrator/ (LangGraph, corre en cualquier host con red)
       ├─ HTTP → server.py (Gemma): percepción semántica + razonamiento de planificación en texto
       └─ rosbridge (websocket, roslibpy) → ROS2 (otro repo)
            ├─ /robot/get_camera_image   (RGB actual de la RealSense)
            ├─ /robot/perceive           (SAM2 + depth: mask, cloud, pose desde box/puntos del VLM)
            ├─ /robot/grasp | pick | place | pick_place   (skills implementadas)
            └─ /robot/push | handover    (declaradas, no implementadas → rechazadas en select_skill)
```

## Estructura de archivos nueva

```
/workspace
  server.py                  # modificar (ver §1)
  orchestrator/
    __init__.py
    config.py                # URLs (VLM server, rosbridge), max_attempts, registry de skills
    state.py                 # GraphState (TypedDict), enums FailureType/Stage, modelos pydantic
    vlm_client.py            # cliente HTTP del servidor VLM (/perceive_scene, /plan_text)
    ros_bridge.py            # cliente roslibpy: get_image, perceive, call_skill
    memory.py                # memoria episódica persistente (JSONL)
    nodes.py                 # nodos del grafo
    graph.py                 # construcción del StateGraph
    main.py                  # CLI: python -m orchestrator.main "pon el pato en la caja"
  contracts/
    ROS2_INTERFACES.md       # contrato de servicios para el repo ROS2
  tests/
    test_graph.py            # grafo con VLM y bridge mockeados
    test_replan_rules.py     # reglas de replanificación y limpieza de caché
  requirements-orchestrator.txt  # langgraph, roslibpy, pydantic, requests
```

## §1. Cambios en server.py (VLM = solo percepción + razonamiento texto)

1. **Nuevo endpoint `/perceive_scene`** (multipart imagen + instrucción): nuevo system prompt multi-objeto que devuelve:
   ```json
   {
     "task": {"skill": "pick_place", "object": "pato", "target": "caja"},
     "objects": [
       {"name": "pato", "visible": true,
        "object_box_xyxy_norm": [..], "positive_points_xy_norm": [..],
        "negative_points_xy_norm": [..], "selected_safe_part": "...",
        "forbidden_parts": [...], "force_level": "low", "confidence": 0.91}
     ]
   }
   ```
   Sin `mask_id` ni `pose_estimate` — eso lo agrega `/robot/perceive` (ROS2). Reusar `run_inference`/`extract_json` existentes parametrizando el system prompt. Mantener `/infer` e `/infer_image` actuales por compatibilidad.
2. **Nuevo endpoint `/plan_text`** (JSON `{prompt}`, sin imagen): inferencia texto-solo con Gemma para `select_skill`/`replan`. Mismo `run_inference` con `messages` sin bloque imagen.
3. `confidence` pasa de categórico a float 0–1 en el prompt nuevo (coincide con el formato pedido).

## §2. Contrato ROS2 (contracts/ROS2_INTERFACES.md)

Documentar (para implementar en el repo ROS2 con `rosbridge_server`):

- **`/robot/get_camera_image`** → `{image_base64, width, height, frame_id, stamp}`.
- **`/robot/perceive`** — request: salida del VLM (objects con box/puntos) → response: objetos enriquecidos `{name, mask_id, pose_estimate: [x,y,z,qx,qy,qz,qw], cloud_points, segmentation_ok}`.
- **`/robot/<skill>`** (grasp, pick, place, pick_place) — request:
  ```
  object_name, target_location (solo place/pick_place),
  clear_cached_grasps: bool, force_new_perception: bool,
  vlm_context_json: string  # box/puntos/forbidden_parts/force_level serializados
  ```
  response:
  ```
  success: bool
  stage: perception|segmentation|grasp_generation|collision_filter|ik|execution|gripper|place
  failure_type: "" | object_not_found | segmentation_failed | no_grasps_generated |
                no_collision_free_grasps | ik_failed | execution_failed |
                gripper_failed | object_slipped | place_failed
  grasp_id, mask_id, detail_json
  ```
- Nota explícita del contrato: cuando `clear_cached_grasps=true`, la skill ROS2 **debe invalidar `top_grasps_cached`** y re-ejecutar GraspGen; con `force_new_perception=true` debe re-segmentar con SAM2.

## §3. Orquestador LangGraph

**Estado** (`state.py`): `instruction`, `task` (skill/object/target), `perception` (objetos enriquecidos), `attempt_number`, `history: list[EpisodeRecord]`, `last_failure: {stage, failure_type, grasp_id, mask_id}`, `skill_args`, `outcome`.

**Grafo** (`graph.py`), flujo exacto pedido:

```
START → parse_instruction → query_vlm_perception → select_skill
      → execute_ros_skill → evaluate_result
          ├─ éxito → save_success → END
          └─ fallo → save_failure → replan ─┬→ query_vlm_perception (si necesita nueva percepción)
                                            ├→ execute_ros_skill (reintento directo)
                                            └→ END (max_attempts o irrecuperable)
```

**Nodos** (`nodes.py`):
- `parse_instruction`: llama `/plan_text` con la instrucción → `{skill, object, target}`; valida contra el registry (push/handover → outcome `skill_not_available`, no se ejecutan).
- `query_vlm_perception`: `ros_bridge.get_image()` → `vlm_client.perceive_scene()` → `ros_bridge.perceive()` → estado visual estructurado completo. Si el objeto no es visible/confianza baja → fallo `object_not_found`.
- `select_skill`: arma `skill_args`; consulta `/plan_text` con instrucción + estado visual + historial + último fallo, y **valida determinísticamente** la respuesta (skill ∈ registry, objeto ∈ percepción).
- `execute_ros_skill`: `ros_bridge.call_skill(skill, args)` con timeout.
- `evaluate_result`: ruteo por `success`.
- `save_success` / `save_failure`: persisten en `memory.py` el registro episódico con **instrucción original, objeto objetivo, skill, etapa, tipo de fallo, grasp usado, máscara usada, número de intento** + timestamp.
- `replan`: política determinista primero, Gemma para desempates:

| Fallo | Acción de replan |
|---|---|
| object_not_found, segmentation_failed | nueva percepción completa (`force_new_perception=true`) |
| no_grasps_generated, no_collision_free_grasps | nueva percepción + pedir al VLM una `selected_safe_part` distinta |
| ik_failed, execution_failed | reintento con `clear_cached_grasps=true` |
| gripper_failed, object_slipped | nueva percepción + reducir `force_level` no — ajustar estrategia de agarre (otra zona) |
| place_failed | reintentar solo `place` si el objeto sigue agarrado; si no, `pick_place` completo |

**Regla crítica (invariante en código, no en el LLM)**: si el último fallo proviene de `grasp`/`pick`/`pick_place`, el siguiente `execute_ros_skill` lleva **siempre** `clear_cached_grasps=true` y `force_new_perception=true`, sin importar lo que sugiera Gemma. Implementada como guard en `replan`/`select_skill` y cubierta por test.

`max_attempts` configurable (default 3); al agotarse → `save_failure` final con outcome `aborted` → END.

## §4. Verificación

1. **Unit tests** (`pytest tests/`): grafo con `vlm_client` y `ros_bridge` mockeados — éxito directo, fallo→replan→éxito, invariante de limpieza de caché tras fallo de grasp, tope de reintentos, rechazo de push/handover.
2. **Integración VLM real**: levantar `server.py`, probar `/perceive_scene` con [mesa.png](/workspace/mesa.png) e instrucción "pon el pato en la caja"; verificar JSON con `task` + `objects`.
3. **End-to-end simulado**: script/fixture con un rosbridge falso (servidor websocket mínimo o stub de roslibpy) que simula respuestas de skills incluyendo fallos, y correr `python -m orchestrator.main "..."` revisando el JSONL de memoria.
4. La integración con el robot real queda en el repo ROS2, guiada por `contracts/ROS2_INTERFACES.md`.

## Dependencias

`requirements-orchestrator.txt`: `langgraph`, `roslibpy`, `pydantic`, `requests`. El orquestador no necesita GPU. Opcional: añadir estos paquetes al Dockerfile o documentar que corre fuera del contenedor.

## Orden de implementación

1. `contracts/ROS2_INTERFACES.md` (fija los contratos).
2. Cambios en `server.py` (`/perceive_scene`, `/plan_text`).
3. `orchestrator/`: state → clients → memory → nodes → graph → main.
4. Tests + verificación con mesa.png.
