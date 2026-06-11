#!/usr/bin/env python3
"""Skill server del UR5.

Expone una capa de "robot skills" de alto nivel como servicios ROS, que orquestan
los servicios de bajo nivel ya existentes (BT node + move_group_client + SAM):

    /robot/grasp     -> /ur5/prepare_bt_grasp_cycle  (SAM select + GraspGen)
    /robot/pick      -> /ur5/execute_pick            (approach->grasp->close->lift)
    /robot/place     -> /ur5/execute_place           (deposito->open->home)
    /robot/pick_place-> pick seguido de place (conveniencia)
    /robot/push      -> stub (por definir)
    /robot/handover  -> stub (por definir)

Todos los servicios usan std_srvs/Trigger. Pensado para usarse desde un script con el
wrapper RobotSkills (robot_skills.py) o directamente con `ros2 service call`.
"""

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class UR5SkillServer(Node):
    def __init__(self):
        super().__init__("ur5_skill_server")

        # Grupo reentrante: los handlers bloquean esperando servicios downstream,
        # así que el executor debe poder procesar otras callbacks en paralelo.
        self.cb_group = ReentrantCallbackGroup()

        # --- Parametros: nombres de servicios downstream ---
        self.grasp_service_name = self.declare_parameter(
            "grasp_service_name", "/ur5/prepare_bt_grasp_cycle"
        ).value
        self.pick_service_name = self.declare_parameter(
            "pick_service_name", "/ur5/execute_pick"
        ).value
        self.place_service_name = self.declare_parameter(
            "place_service_name", "/ur5/execute_place"
        ).value

        # --- Parametros: nombres de los skills expuestos ---
        skill_grasp = self.declare_parameter("skill_grasp_name", "/robot/grasp").value
        skill_pick = self.declare_parameter("skill_pick_name", "/robot/pick").value
        skill_place = self.declare_parameter("skill_place_name", "/robot/place").value
        skill_pick_place = self.declare_parameter(
            "skill_pick_place_name", "/robot/pick_place"
        ).value
        skill_push = self.declare_parameter("skill_push_name", "/robot/push").value
        skill_handover = self.declare_parameter(
            "skill_handover_name", "/robot/handover"
        ).value

        # --- Timeouts ---
        self.service_wait_timeout_sec = float(
            self.declare_parameter("service_wait_timeout_sec", 5.0).value
        )
        self.grasp_timeout_sec = float(
            self.declare_parameter("grasp_timeout_sec", 120.0).value
        )
        self.pick_timeout_sec = float(
            self.declare_parameter("pick_timeout_sec", 180.0).value
        )
        self.place_timeout_sec = float(
            self.declare_parameter("place_timeout_sec", 180.0).value
        )

        # --- Placeholders para skills por definir ---
        self.push_distance = float(self.declare_parameter("push_distance", 0.10).value)
        self.push_direction = list(
            self.declare_parameter("push_direction", [1.0, 0.0, 0.0]).value
        )
        self.handover_joint_positions = list(
            self.declare_parameter("handover_joint_positions", []).value
        )

        # --- Clientes downstream ---
        self.grasp_client = self.create_client(
            Trigger, self.grasp_service_name, callback_group=self.cb_group
        )
        self.pick_client = self.create_client(
            Trigger, self.pick_service_name, callback_group=self.cb_group
        )
        self.place_client = self.create_client(
            Trigger, self.place_service_name, callback_group=self.cb_group
        )

        # --- Servicios de skills ---
        self.create_service(
            Trigger, skill_grasp, self._handle_grasp, callback_group=self.cb_group
        )
        self.create_service(
            Trigger, skill_pick, self._handle_pick, callback_group=self.cb_group
        )
        self.create_service(
            Trigger, skill_place, self._handle_place, callback_group=self.cb_group
        )
        self.create_service(
            Trigger,
            skill_pick_place,
            self._handle_pick_place,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger, skill_push, self._handle_push, callback_group=self.cb_group
        )
        self.create_service(
            Trigger, skill_handover, self._handle_handover, callback_group=self.cb_group
        )

        self.get_logger().info(
            "ur5_skill_server listo. Skills: "
            f"{skill_grasp}, {skill_pick}, {skill_place}, {skill_pick_place}, "
            f"{skill_push} (stub), {skill_handover} (stub)."
        )

    # ------------------------------------------------------------------
    # Utilidades
    # ------------------------------------------------------------------
    def _wait_for_future_result(self, future, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return None
        return future.result()

    def _call_trigger(self, client, service_name: str, timeout_sec: float, label: str):
        if not client.wait_for_service(timeout_sec=self.service_wait_timeout_sec):
            msg = f"Servicio '{service_name}' no disponible"
            self.get_logger().error(f"Skill: {msg} para {label}.")
            return False, msg

        future = client.call_async(Trigger.Request())
        response = self._wait_for_future_result(future, timeout_sec=timeout_sec)
        if response is None:
            msg = f"Timeout en '{service_name}' durante {label}"
            self.get_logger().error(f"Skill: {msg}.")
            return False, msg

        if not response.success:
            self.get_logger().warn(
                f"Skill: '{service_name}' fallo durante {label}: {response.message}"
            )
            return False, response.message

        self.get_logger().info(
            f"Skill: '{service_name}' completado para {label}: {response.message}"
        )
        return True, response.message

    # ------------------------------------------------------------------
    # Skills
    # ------------------------------------------------------------------
    def _handle_grasp(self, request, response):
        del request
        ok, message = self._call_trigger(
            self.grasp_client,
            self.grasp_service_name,
            self.grasp_timeout_sec,
            "grasp (SAM select + GraspGen)",
        )
        response.success = ok
        response.message = message
        return response

    def _handle_pick(self, request, response):
        del request
        ok, message = self._call_trigger(
            self.pick_client,
            self.pick_service_name,
            self.pick_timeout_sec,
            "pick (toma y levanta)",
        )
        response.success = ok
        response.message = message
        return response

    def _handle_place(self, request, response):
        del request
        ok, message = self._call_trigger(
            self.place_client,
            self.place_service_name,
            self.place_timeout_sec,
            "place (deposito)",
        )
        response.success = ok
        response.message = message
        return response

    def _handle_pick_place(self, request, response):
        del request
        ok, message = self._call_trigger(
            self.pick_client,
            self.pick_service_name,
            self.pick_timeout_sec,
            "pick (toma y levanta)",
        )
        if not ok:
            response.success = False
            response.message = f"pick_place abortado en pick: {message}"
            return response

        ok, message = self._call_trigger(
            self.place_client,
            self.place_service_name,
            self.place_timeout_sec,
            "place (deposito)",
        )
        response.success = ok
        response.message = (
            "pick_place completado" if ok else f"pick_place fallo en place: {message}"
        )
        return response

    def _handle_push(self, request, response):
        del request
        # TODO(skills): implementar push real. Idea: a partir de la nube/objeto
        # segmentado, planificar un approach lateral y un movimiento cartesiano de
        # `push_distance` en la direccion `push_direction`, sin cerrar el gripper.
        self.get_logger().warn(
            "Skill push solicitada pero aun no esta implementada (por definir)."
        )
        response.success = False
        response.message = (
            "push no implementado (por definir): "
            f"distance={self.push_distance}, direction={self.push_direction}"
        )
        return response

    def _handle_handover(self, request, response):
        del request
        # TODO(skills): implementar handover real. Idea: tras un pick exitoso, mover
        # a `handover_joint_positions` y abrir el gripper al detectar fuerza/contacto
        # del humano (o tras un temporizador).
        self.get_logger().warn(
            "Skill handover solicitada pero aun no esta implementada (por definir)."
        )
        response.success = False
        response.message = (
            "handover no implementado (por definir): "
            f"handover_joint_positions={self.handover_joint_positions}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = UR5SkillServer()
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
