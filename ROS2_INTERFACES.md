# Contrato de interfaces ROS2 ↔ Orquestador LangGraph

Este documento define los servicios que el workspace ROS2 (repo del robot: UR5 +
Robotiq Hand-E + RealSense + SAM2 + GraspGen + MoveIt2) debe exponer a través de
`rosbridge_server` (websocket, JSON). El orquestador (`orchestrator/` en este
repo) los consume con `roslibpy`.

Todos los servicios usan tipos serializables a JSON vía rosbridge. El contrato
vinculante es la forma JSON descrita aquí.

**Implementación de referencia:** paquete `ur5_manipulation` (ROS2 `ament_cmake` +
`rosidl`), nodos `robot_perception_node` y `robot_skill_node`, tipos custom:

| Servicio | Tipo |
|---|---|
| `/robot/get_camera_image` | `ur5_manipulation/srv/GetCameraImage` |
| `/robot/perceive` | `ur5_manipulation/srv/Perceive` |
| `/robot/grasp` \| `pick` \| `place` \| `pickplace` \| `push` \| `handover` | `ur5_manipulation/srv/RunSkill` |

Lanzar con: `ros2 launch ur5_manipulation robot_services.launch.py` (incluye
`rosbridge_websocket` en el puerto 9090).

> **Nota de codificación (importante para el orquestador):** los campos de
> `/robot/perceive` que contienen arreglos anidados de objetos viajan como
> **strings JSON** (`objects_json`), no como arreglos nativos ROS, porque los
> mensajes ROS no permiten arreglos de arreglos (ej. `positive_points_xy_norm:
> [[x,y],...]`). El orquestador debe `json.dumps` la lista de objetos al enviar y
> `json.loads` la respuesta. Los demás servicios usan campos planos nativos.

---

## 1. `/robot/get_camera_image`

Captura el frame RGB actual de la RealSense para enviarlo al servidor VLM.

**Request:** `{}` (vacío)

**Response:**

```json
{
  "success": true,
  "image_base64": "<JPEG o PNG codificado en base64>",
  "encoding": "jpeg",
  "width": 1280,
  "height": 720,
  "frame_id": "camera_color_optical_frame",
  "stamp": 1759999999.123
}
```

Notas:
- La imagen debe corresponder al mismo instante (o muy cercano) del depth frame
  que usará `/robot/perceive`, para que los puntos/box del VLM sean consistentes.
- `frame_id` y `stamp` permiten al lado ROS2 asociar la percepción VLM con el
  par RGB-D correcto.

---

## 2. `/robot/perceive`

Enriquece la salida semántica del VLM (box + puntos normalizados) con SAM2 y
depth para producir máscara, nube de puntos y pose aproximada por objeto.

**Request:** (`objects_json` es la lista serializada con `json.dumps`)

```json
{
  "stamp": 1759999999.123,
  "objects_json": "[{\"name\": \"pato\", \"object_box_xyxy_norm\": [0.41, 0.32, 0.58, 0.51], \"positive_points_xy_norm\": [[0.49, 0.41]], \"negative_points_xy_norm\": [[0.52, 0.48]], \"selected_safe_part\": \"cuerpo\", \"forbidden_parts\": [\"cabeza\"], \"force_level\": \"low\"}]"
}
```

Cada elemento dentro de `objects_json`:

```json
{
  "name": "pato",
  "object_box_xyxy_norm": [0.41, 0.32, 0.58, 0.51],
  "positive_points_xy_norm": [[0.49, 0.41]],
  "negative_points_xy_norm": [[0.52, 0.48]],
  "selected_safe_part": "cuerpo",
  "forbidden_parts": ["cabeza"],
  "force_level": "low"
}
```

- `stamp`: el devuelto por `/robot/get_camera_image`, para reusar el frame RGB-D
  correcto. Si el nodo no puede recuperar ese frame, debe capturar uno nuevo y
  reportarlo en `detail`.
- Coordenadas normalizadas en [0, 1] respecto a la imagen RGB entregada.

**Response:** (`objects_json` es la lista serializada; haz `json.loads`)

```json
{
  "success": true,
  "objects_json": "[{\"name\": \"pato\", \"segmentation_ok\": true, \"mask_id\": \"mask_01\", \"pose_estimate\": [0.42, -0.10, 0.06, 0.0, 0.0, 0.0, 1.0], \"cloud_topic\": \"/perception/pato/points\", \"num_cloud_points\": 4231}]",
  "failure_type": "",
  "detail": ""
}
```

Cada elemento dentro de `objects_json`:

```json
{
  "name": "pato",
  "segmentation_ok": true,
  "mask_id": "mask_01",
  "pose_estimate": [0.42, -0.10, 0.06, 0.0, 0.0, 0.0, 1.0],
  "cloud_topic": "/perception/pato/points",
  "num_cloud_points": 4231
}
```

- `mask_id`: identificador con el que el lado ROS2 cachea la máscara SAM2. Las
  skills lo reciben de vuelta vía `vlm_context_json` y lo reportan en su response.
- `pose_estimate`: `[x, y, z, qx, qy, qz, qw]` en el frame base del robot
  (ej. `base_link`). Es una pose aproximada (centroide + orientación principal);
  la pose de agarre final la decide GraspGen.
- `cloud_topic` (o alternativamente `cloud_points` inline si es pequeña): cómo
  acceder a la nube de puntos segmentada.
- Si un objeto no se pudo segmentar: `segmentation_ok: false` y
  `failure_type: "segmentation_failed"` a nivel de response si falló todo.

---

## 3. Skills: `/robot/grasp`, `/robot/pick`, `/robot/place`, `/robot/pick_place`

Cada skill encapsula el pipeline completo de bajo nivel:
RealSense → SAM2 → GraspGen → filtro de colisión → MoveIt2 (IK + plan) →
ejecución UR5 → gripper Robotiq Hand-E.

`/robot/push` y `/robot/handover` quedan declaradas pero **no implementadas**;
si se llaman deben responder `success: false`, `failure_type: "skill_not_available"`.

**Request (común):**

```json
{
  "object_name": "pato",
  "target_location": "caja",
  "clear_cached_grasps": true,
  "force_new_perception": true,
  "vlm_context_json": "{\"name\":\"pato\",\"mask_id\":\"mask_01\",\"object_box_xyxy_norm\":[...],\"positive_points_xy_norm\":[...],\"negative_points_xy_norm\":[...],\"selected_safe_part\":\"cuerpo\",\"forbidden_parts\":[\"cabeza\"],\"force_level\":\"low\",\"pose_estimate\":[...]}",
  "attempt_number": 2
}
```

- `target_location`: solo relevante para `place` y `pick_place`; vacío en el resto.
- `vlm_context_json`: contexto de percepción serializado (salida VLM + datos de
  `/robot/perceive`) para que la skill no necesite volver a consultar al VLM.
- `attempt_number`: número de intento (1-indexado) para logging del lado ROS2.

### Modo sin VLM

Si `vlm_context_json` llega **vacío** (o sin `object_box_xyxy_norm` válido),
`robot_skill_node` prepara con el ciclo **sin VLM**
(`/ur5/prepare_bt_grasp_cycle`, selección de objeto vía `/sam2/select_object`)
en vez del ciclo VLM (`/ur5/prepare_vlm_bt_grasp_cycle`). Esto permite ejecutar
las skills directamente sin el servidor VLM. En ese modo
`force_new_perception: true` no re-publica prompt (no hay); el propio ciclo de
preparación re-segmenta en cada llamada. El modo usado se reporta en
`detail_json` como `"prepare_mode": "vlm" | "no_vlm"`.

### Formato de respuesta del servidor VLM (HTTP `/infer_image`)

El cliente/orquestador consume el formato anidado del servidor VLM actual:

```json
{
  "success": true,
  "result": {
    "task": {"skill": "pick_place", "object": "barco", "target": "caja"},
    "objects": [
      {
        "name": "small blue plastic item",
        "confidence": 0.95,
        "visible": true,
        "object_box_xyxy_norm": [0.53, 0.35, 0.65, 0.51],
        "positive_points_xy_norm": [[0.58, 0.43]],
        "negative_points_xy_norm": [[0.53, 0.35], [0.65, 0.51]],
        "selected_safe_part": "body",
        "forbidden_parts": ["plastic"],
        "force_level": "low"
      }
    ]
  }
}
```

- `task` aporta la decisión (`skill`, `object` → `object_name`,
  `target` → `target_location`).
- Cada elemento de `objects` ya tiene el esquema por-objeto de
  `/robot/perceive`; los objetos con `visible: true` y box válido se pasan tal
  cual en `objects_json`, y el que corresponde a `task.object` (match por
  nombre; si no calza, mayor `confidence` visible) se serializa como
  `vlm_context_json` para la skill.
- Se mantiene compatibilidad con el formato plano anterior (`skill`,
  `target_object`, `target_location` y el objeto al nivel raíz del `result`).

### Semántica OBLIGATORIA de los flags

- `clear_cached_grasps: true` → la skill **debe invalidar cualquier caché de
  agarres** (p. ej. `top_grasps_cached`) y volver a ejecutar GraspGen sobre una
  segmentación fresca. **Nunca** reutilizar grasps de un intento anterior.
- `force_new_perception: true` → la skill debe re-capturar RGB-D y re-segmentar
  con SAM2 (puede usar los prompts de `vlm_context_json`), ignorando máscaras
  cacheadas.
- El orquestador SIEMPRE envía ambos flags en `true` en reintentos posteriores a
  un fallo de `grasp`/`pick`/`pick_place`. El lado ROS2 debe respetarlos aunque
  tenga caché válida.

**Response (común):**

```json
{
  "success": false,
  "stage": "ik",
  "failure_type": "ik_failed",
  "grasp_id": "grasp_017",
  "mask_id": "mask_01",
  "detail_json": "{\"planner\":\"RRTConnect\",\"attempts\":5,\"grasp_score\":0.83}"
}
```

- `stage`: etapa alcanzada, una de:
  `perception | segmentation | grasp_generation | collision_filter | ik | execution | gripper | place | done`
- `failure_type`: `""` si `success: true`; si no, una de:

| failure_type | Significado | Etapa típica |
|---|---|---|
| `object_not_found` | El objeto no se encontró en la escena | perception |
| `segmentation_failed` | SAM2 no produjo máscara válida | segmentation |
| `no_grasps_generated` | GraspGen no devolvió candidatos | grasp_generation |
| `no_collision_free_grasps` | Todos los grasps filtrados por colisión | collision_filter |
| `ik_failed` | MoveIt2 no encontró IK/plan para ningún grasp | ik |
| `execution_failed` | Falló la ejecución de trayectoria en el UR5 | execution |
| `gripper_failed` | El gripper no cerró/abrió correctamente | gripper |
| `object_slipped` | El objeto se soltó tras el agarre | execution |
| `place_failed` | Falló la colocación en el destino | place |
| `skill_not_available` | Skill declarada pero no implementada | - |

- `grasp_id` / `mask_id`: identificadores del grasp y la máscara usados en el
  intento (aunque haya fallado), para la memoria episódica del orquestador.
- `detail_json`: información libre adicional (scores, nombre del planner, etc.).

> **Origen del `failure_type` (implementación):** no se infiere parseando texto.
> Los nodos downstream que detectan el fallo (`behavior_tree_node` para
> segmentación/grasp_generation; `move_group_client` para
> collision_filter/ik/execution/gripper/slip/place) publican un código
> **estructurado** `{failure_type, stage, detail, stamp}` en el topic latcheado
> `/ur5/skill_failure` (`std_msgs/String` JSON, `transient_local`).
> `robot_skill_node` lee el último valor (con `stamp >= inicio_de_la_llamada`) y
> lo reenvía tal cual en la respuesta. Solo si ese código no llega a tiempo se
> usa una heurística de respaldo sobre el `message`.

### Específico por skill

| Skill | Hace | Campos extra |
|---|---|---|
| `/robot/grasp` | Acercarse y cerrar gripper sobre el objeto (sin levantar) | — |
| `/robot/pick` | grasp + levantar a pose de retract | — |
| `/robot/place` | Llevar el objeto (ya agarrado) a `target_location` y soltar | request: `object_in_gripper: true` esperado |
| `/robot/pick_place` | pick + place completos | requiere `target_location` |

`/robot/place` además debe reportar en `detail_json` el campo
`"object_still_grasped": true|false` cuando falle, para que el orquestador
decida entre reintentar solo `place` o un `pick_place` completo.

---

## 4. Requisitos del lado ROS2

1. `rosbridge_server` (websocket) accesible en `ws://<robot-host>:9090`.
2. Los servicios anteriores registrados antes de aceptar tareas.
3. Timeouts: las skills deben responder siempre (éxito o fallo) en < 120 s;
   el orquestador aborta el intento pasado ese tiempo y lo registra como
   `execution_failed`.
4. Seguridad: las skills son responsables de los límites de velocidad/fuerza,
   E-stop y validación de workspace. El orquestador nunca envía poses ni
   trayectorias crudas: solo nombres de objetos y flags.
|