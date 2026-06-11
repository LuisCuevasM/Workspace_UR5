# VLM Docker — CUDA OOM al recargar SAM2

## Error reportado

```
[WARN] [sam2_service_node]: No se pudo correr SAM2 video predictor:
CUDA out of memory. Tried to allocate 112.00 MiB.
GPU 0 has a total capacity of 11.59 GiB of which 103.12 MiB is free.
Including non-PyTorch memory, this process has 1.05 GiB memory in use.
Process 82271 has 182.00 MiB memory in use.
Of the allocated memory 794.84 MiB is allocated by PyTorch,
and 51.16 MiB is reserved by PyTorch but unallocated.
```

## Diagnóstico

### Secuencia de nvidia-smi

| Momento | GPU usada | Detalle |
|---|---|---|
| Antes de VLM | 442 MiB | Solo rviz2 |
| Durante inferencia VLM | 11 014 MiB | rviz2 + 2× python3 |
| Tras llamar `/release` | **11 748 MiB** | rviz2: 20 MiB, python3×2: 1 254 MiB |
| SAM2 intenta correr | OOM — 103 MiB libres | — |

### Causa raíz — dos problemas independientes

#### Problema 1 — `torch_dtype="auto"` carga float32 (~10 GB)

El modelo `google/gemma-4-E2B-it` con `"auto"` selecciona float32 en capas que no tienen un kernel bfloat16 registrado. Resultado:

```
2B parámetros × 4 bytes (fp32) = ~8 GB pesos
+ activaciones de inferencia   = ~2-3 GB extra
                                ≈ 10-11 GB totales
```

Con `bfloat16` forzado el mismo modelo usa ≈ 4-5 GB.

#### Problema 2 — El contexto CUDA del contenedor Docker retiene memoria tras `/release`

Después de llamar al endpoint `/release`, el servidor reporta `cuda_allocated_gb ≈ 0`
(lo que PyTorch ve dentro del contenedor). Pero nvidia-smi desde el host muestra:

```
Total GPU usado:   11 748 MiB
Procesos visibles: rviz2(20) + python3×2(1 254) = 1 274 MiB
Diferencia:        ~10 474 MiB  ← contexto CUDA del contenedor Docker
```

**Por qué ocurre:** `torch.cuda.empty_cache()` libera el cache del allocator de PyTorch, pero el driver CUDA retiene páginas físicas a nivel de contexto hasta que el proceso termina. Con `device_map="auto"` de accelerate, los hooks `AlignDevicesHook` además pueden mantener referencias circulares que el GC no resuelve en una sola pasada.

**Por qué el BT no lo detectaba:** el método `_wait_vlm_vram_released()` solo consultaba
el endpoint `/health` del servidor (que reporta `cuda_allocated_gb` interno de PyTorch).
Al pasar el umbral de 1.5 GB el BT procedía a cargar SAM2 aunque la GPU real seguía llena.

---

## Cambios a implementar en el Docker VLM

Archivo a modificar: `vlm_gemma_server.py`

### Cambio 1 — Variables de entorno al inicio del archivo

**Antes:**
```python
os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")
```

**Después:**
```python
os.environ.setdefault(
    "PYTORCH_CUDA_ALLOC_CONF",
    "expandable_segments:True,garbage_collection_threshold:0.8",
)
os.environ.setdefault("PYTORCH_NO_CUDA_MEMORY_CACHING", "1")
```

**Por qué:**
- `garbage_collection_threshold:0.8` dispara el GC del allocator cuando el 80 % de la
  memoria reservada está en uso, liberando bloques sin esperar a `empty_cache()`.
- `PYTORCH_NO_CUDA_MEMORY_CACHING=1` hace que PyTorch llame `cudaFree()` directamente
  al destruir tensores, en lugar de guardarlos en cache. Esto devuelve páginas al driver
  de forma inmediata y es lo que permite que nvidia-smi refleje la liberación real.
  **Debe declararse antes de `import torch`.**

---

### Cambio 2 — `init_model()`: forzar bfloat16

**Antes:**
```python
model = AutoModelForImageTextToText.from_pretrained(
    MODEL_ID,
    torch_dtype="auto",
    device_map="auto",
)
```

**Después:**
```python
model = AutoModelForImageTextToText.from_pretrained(
    MODEL_ID,
    torch_dtype=torch.bfloat16,   # "auto" puede usar fp32 (~10 GB); bf16 usa ~4-5 GB
    device_map="auto",
)
```

**Por qué:** `"auto"` puede resolver a float32. `bfloat16` es el formato nativo de Ampere/Ada
para inferencia: misma velocidad, la mitad de VRAM. Reduce el pico de 11 GB a ~4-5 GB.

---

### Cambio 3 — `free_gpu_model()`: cleanup más agresivo

**Antes:**
```python
if model is not None:
    try:
        from accelerate.hooks import remove_hook_from_module
        remove_hook_from_module(model, recurse=True)
    except Exception:
        traceback.print_exc()

    try:
        model.generation_config = None
    except Exception:
        pass

model = None
processor = None

gc.collect()

if torch.cuda.is_available():
    torch.cuda.empty_cache()
    torch.cuda.ipc_collect()
    torch.cuda.synchronize()
```

**Después:**
```python
if model is not None:
    # Quitar hooks de accelerate: AlignDevicesHook mantiene refs a tensores
    # en GPU; sin esto del model no baja el refcount a 0.
    try:
        from accelerate.hooks import remove_hook_from_module
        remove_hook_from_module(model, recurse=True)
    except Exception:
        traceback.print_exc()

    # Borrar atributos que pueden retener tensores.
    for attr in ("generation_config", "_cache", "past_key_values"):
        try:
            setattr(model, attr, None)
        except Exception:
            pass

    # Mover pesos a CPU antes de soltar: obliga a CUDA a liberar páginas activas
    # en drivers que no las devuelven al pool con empty_cache solamente.
    try:
        model.cpu()
    except Exception:
        pass

model = None
processor = None

# Dos pasadas: la primera libera objetos, la segunda finaliza los que tenían
# referencias circulares descubiertas en la primera (común en módulos anidados).
gc.collect()
gc.collect()

if torch.cuda.is_available():
    torch.cuda.empty_cache()
    torch.cuda.ipc_collect()
    torch.cuda.synchronize()
```

**Por qué cada adición:**
| Adición | Motivo |
|---|---|
| `_cache`, `past_key_values` en el loop | Cachés de KV y de generación que `transformers` puede dejar vivos tras `generate()` |
| `model.cpu()` | Mueve tensores activos a RAM antes de que el GC los destruya; evita que el driver los retenga en "reserved but unallocated" |
| Doble `gc.collect()` | Python finaliza en la segunda pasada objetos con `__del__` que quedaron en la lista de finalización tras la primera |

---

## Cómo aplicar los cambios en el contenedor

```bash
# Copiar el archivo actualizado al contenedor (reemplaza <nombre> por el tuyo)
docker cp ur_ws/src/ur5_manipulation/scripts/vlm_gemma_server.py <nombre_contenedor>:/ruta/en/docker/vlm_gemma_server.py

# Reiniciar el servidor dentro del contenedor (mata el proceso anterior)
docker exec <nombre_contenedor> pkill -f vlm_gemma_server.py
docker exec -d <nombre_contenedor> python3 /ruta/en/docker/vlm_gemma_server.py
```

O si usas docker compose:
```bash
docker compose restart <servicio_vlm>
```

---

## Resultado esperado tras los cambios

| Métrica | Antes | Después |
|---|---|---|
| VRAM durante inferencia | ~11 GB (fp32) | ~4-5 GB (bf16) |
| VRAM tras `/release` (nvidia-smi) | ~10.5 GB retenidos | < 1-2 GB (contexto mínimo) |
| GPU libre para SAM2 | 103 MiB → OOM | > 9 GB → sin OOM |
| `/health` → `cuda_allocated_gb` tras release | ~0 (engañoso) | ~0 (coherente con host) |

---

## Cambios complementarios en el BT (ya aplicados en el repo)

El BT también fue corregido para no depender del auto-reporte del contenedor:

- `_get_gpu_free_gb()` — consulta nvidia-smi desde el host (ve la memoria real)
- `_wait_vlm_vram_released()` — ahora exige `gpu_free ≥ vlm_min_free_vram_gb` (default 4 GB)
  además de `model_loaded=False`
- `_call_vlm_with_lifecycle()` — ahora **aborta** si no se libera suficiente VRAM en lugar
  de continuar hacia el OOM con solo un warning

Estos cambios en el BT son una red de seguridad independiente de los cambios en Docker.
Ambas capas juntas garantizan que SAM2 nunca se cargue mientras el VLM siga ocupando la GPU.
