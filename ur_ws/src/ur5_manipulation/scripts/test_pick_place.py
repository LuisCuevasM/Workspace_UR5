#!/usr/bin/env python3
"""
Test de pick and place para un objeto específico.

Modos de uso:
  # Pipeline completo (VLM decide skill y objeto):
  ros2 run ur5_manipulation test_pick_place.py --instruction "agarra el pato y colócalo en la caja"

  # Pipeline completo con instrucción por defecto:
  ros2 run ur5_manipulation test_pick_place.py

  # Solo consulta al VLM sin ejecutar el robot:
  ros2 run ur5_manipulation test_pick_place.py --dry-run

  # Llamada directa al skill sin VLM (objeto conocido):
  ros2 run ur5_manipulation test_pick_place.py --direct --object pato
"""
import argparse
import json
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from ur5_manipulation.srv import ExecuteInstruction, QueryInstruction, RunSkill


DEFAULT_INSTRUCTION = "agarra el pato"
TIMEOUT_SEC = 300.0


def _print_section(title: str):
    print(f"\n{'─' * 60}")
    print(f"  {title}")
    print(f"{'─' * 60}")


def _pretty_json(raw: str) -> str:
    if not raw:
        return "(vacío)"
    try:
        return json.dumps(json.loads(raw), indent=2, ensure_ascii=False)
    except Exception:
        return raw


class PickPlaceTester(Node):
    def __init__(self):
        super().__init__("pick_place_tester")

        self._execute_client = self.create_client(
            ExecuteInstruction, "/ur5/execute_instruction"
        )
        self._query_client = self.create_client(
            QueryInstruction, "/ur5/query_instruction"
        )
        self._pick_place_client = self.create_client(
            RunSkill, "/robot/pick_place"
        )

    def _wait(self, client, service_name: str):
        self.get_logger().info(f"Esperando {service_name}...")
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f"Servicio {service_name} no disponible.")
            return False
        return True

    def run_full_pipeline(self, instruction: str) -> bool:
        """Llama /ur5/execute_instruction — VLM decide skill + objeto + ejecuta."""
        if not self._wait(self._execute_client, "/ur5/execute_instruction"):
            return False

        _print_section(f"PIPELINE COMPLETO | instrucción: '{instruction}'")

        req = ExecuteInstruction.Request()
        req.instruction = instruction

        print("Enviando instrucción al pipeline...")
        future = self._execute_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=TIMEOUT_SEC)

        if future.result() is None:
            print("ERROR: sin respuesta (timeout o excepción).")
            return False

        res = future.result()
        self._print_execute_result(res)
        return res.success

    def run_dry_run(self, instruction: str) -> bool:
        """Solo consulta el VLM — sin mover el robot."""
        if not self._wait(self._query_client, "/ur5/query_instruction"):
            return False

        _print_section(f"DRY-RUN (solo VLM) | instrucción: '{instruction}'")

        req = QueryInstruction.Request()
        req.instruction = instruction

        print("Consultando VLM...")
        future = self._query_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=TIMEOUT_SEC)

        if future.result() is None:
            print("ERROR: sin respuesta (timeout o excepción).")
            return False

        res = future.result()
        print(f"\nskill seleccionada : {res.selected_skill}")
        print(f"objeto             : {res.object_name}")
        print(f"target_location    : {res.target_location}")
        print(f"success            : {res.success}")
        print(f"message            : {res.message}")
        print(f"\nvlm_context_json:\n{_pretty_json(res.vlm_context_json)}")
        print(f"\nraw_response_json:\n{_pretty_json(res.raw_response_json)}")
        return res.success

    def run_direct(self, object_name: str, target_location: str = "") -> bool:
        """Llama /robot/pick_place directamente sin pasar por el VLM."""
        if not self._wait(self._pick_place_client, "/robot/pick_place"):
            return False

        _print_section(f"DIRECTO | objeto: '{object_name}' | target: '{target_location or \"(fijo)\"}'")

        req = RunSkill.Request()
        req.object_name = object_name
        req.target_location = target_location
        req.clear_cached_grasps = True
        req.force_new_perception = True
        req.vlm_context_json = ""
        req.attempt_number = 1

        print("Ejecutando pick_place directo...")
        future = self._pick_place_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=TIMEOUT_SEC)

        if future.result() is None:
            print("ERROR: sin respuesta (timeout o excepción).")
            return False

        res = future.result()
        print(f"\nsuccess      : {res.success}")
        print(f"stage        : {res.stage}")
        print(f"failure_type : {res.failure_type}")
        print(f"grasp_id     : {res.grasp_id}")
        print(f"mask_id      : {res.mask_id}")
        print(f"\ndetail_json:\n{_pretty_json(res.detail_json)}")
        return res.success

    def _print_execute_result(self, res):
        ok = "OK" if res.success else "FALLO"
        print(f"\nResultado: {ok}")
        print(f"  selected_skill  : {res.selected_skill}")
        print(f"  executed_skill  : {res.executed_skill}")
        print(f"  service_name    : {res.service_name}")
        print(f"  object_name     : {res.object_name}")
        print(f"  target_location : {res.target_location}")
        print(f"  stage           : {res.stage}")
        print(f"  failure_type    : {res.failure_type}")
        print(f"  message         : {res.message}")
        print(f"\nquery_response_json:\n{_pretty_json(res.query_response_json)}")
        print(f"\ndecode_detail_json:\n{_pretty_json(res.decode_detail_json)}")
        print(f"\nraw_response_json:\n{_pretty_json(res.raw_response_json)}")


def main():
    parser = argparse.ArgumentParser(description="Test pick and place UR5")
    parser.add_argument(
        "--instruction",
        default=DEFAULT_INSTRUCTION,
        help="Instrucción en lenguaje natural (modo pipeline completo)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Solo consulta el VLM, sin mover el robot",
    )
    parser.add_argument(
        "--direct",
        action="store_true",
        help="Llama /robot/pick_place directamente sin VLM",
    )
    parser.add_argument(
        "--object",
        default="pato",
        help="Nombre del objeto para modo --direct (default: pato)",
    )
    parser.add_argument(
        "--target",
        default="",
        help="Ubicación destino para modo --direct (vacío = posición fija)",
    )

    # ros2 run pasa args extra después de '--'
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = PickPlaceTester()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        if args.dry_run:
            success = node.run_dry_run(args.instruction)
        elif args.direct:
            success = node.run_direct(args.object, args.target)
        else:
            success = node.run_full_pipeline(args.instruction)
    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    _print_section("FIN")
    print("Resultado final:", "EXITO" if success else "FALLO")
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
