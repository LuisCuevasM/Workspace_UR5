#!/usr/bin/env python3

import threading
import time
import json

import rclpy
from control_msgs.action import GripperCommand
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger


class Ur5GripperControlNode(Node):
    def __init__(self):
        super().__init__("ur5_gripper_control")

        self.gripper_action_name = self.declare_parameter(
            "gripper_action_name",
            "gripper/gripper_action_controller/gripper_cmd",
        ).value
        self.close_gripper_position = float(
            self.declare_parameter("close_gripper_position", 0.0).value
        )
        self.close_gripper_max_effort = float(
            self.declare_parameter("close_gripper_max_effort", 0.0).value
        )
        self.close_gripper_velocity = float(
            self.declare_parameter("close_gripper_velocity", 0.05).value
        )
        self.close_gripper_force = float(
            self.declare_parameter("close_gripper_force", 1.5).value
        )
        self.open_gripper_position = float(
            self.declare_parameter("open_gripper_position", 0.025).value
        )
        self.open_gripper_max_effort = float(
            self.declare_parameter("open_gripper_max_effort", 0.0).value
        )
        self.open_gripper_velocity = float(
            self.declare_parameter("open_gripper_velocity", 0.05).value
        )
        self.open_gripper_force = float(
            self.declare_parameter("open_gripper_force", 1.0).value
        )
        self.action_timeout_sec = float(
            self.declare_parameter("action_timeout_sec", 10.0).value
        )
        self.velocity_command_topic = self.declare_parameter(
            "velocity_command_topic",
            "/gripper/gripper_velocity_controller/commands",
        ).value
        self.force_command_topic = self.declare_parameter(
            "force_command_topic",
            "/gripper/gripper_force_controller/commands",
        ).value
        self.config_topic = self.declare_parameter(
            "config_topic", "/ur5/gripper/config"
        ).value
        self.open_service_name = self.declare_parameter(
            "open_service_name", "/ur5/gripper/open"
        ).value
        self.close_service_name = self.declare_parameter(
            "close_service_name", "/ur5/gripper/close"
        ).value

        self.cb_group = ReentrantCallbackGroup()
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            self.gripper_action_name,
            callback_group=self.cb_group,
        )
        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            self.velocity_command_topic,
            10,
        )
        self.force_pub = self.create_publisher(
            Float64MultiArray,
            self.force_command_topic,
            10,
        )
        self.create_subscription(
            String,
            self.config_topic,
            self._on_gripper_config,
            10,
            callback_group=self.cb_group,
        )
        self.lock = threading.Lock()

        self.create_service(
            Trigger,
            self.open_service_name,
            self._handle_open,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger,
            self.close_service_name,
            self._handle_close,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            "Nodo de gripper listo. "
            f"action='{self.gripper_action_name}', "
            f"open_service='{self.open_service_name}', "
            f"close_service='{self.close_service_name}', "
            f"velocity_topic='{self.velocity_command_topic}', "
            f"force_topic='{self.force_command_topic}', "
            f"config_topic='{self.config_topic}', "
            f"open_position={self.open_gripper_position:.4f}, "
            f"close_position={self.close_gripper_position:.4f}."
        )

    def _on_gripper_config(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Config de gripper invalida: {exc}")
            return

        allowed = {
            "close_gripper_position",
            "close_gripper_max_effort",
            "close_gripper_velocity",
            "close_gripper_force",
            "open_gripper_position",
            "open_gripper_max_effort",
            "open_gripper_velocity",
            "open_gripper_force",
        }

        params = []
        for name, value in data.items():
            if name not in allowed:
                continue
            try:
                params.append(Parameter(name, Parameter.Type.DOUBLE, float(value)))
            except (TypeError, ValueError):
                self.get_logger().warn(
                    f"Valor invalido para '{name}' en config gripper: {value}"
                )

        if not params:
            self.get_logger().warn("Config de gripper no contenia parametros validos.")
            return

        results = self.set_parameters(params)
        failed = [
            f"{param.name}: {result.reason}"
            for param, result in zip(params, results)
            if not result.successful
        ]
        if failed:
            self.get_logger().warn(
                "No se pudo actualizar toda la config de gripper: "
                + "; ".join(failed)
            )
            return

        applied = ", ".join(f"{param.name}={param.value:.4f}" for param in params)
        self.get_logger().info(f"Config de gripper actualizada desde VLM: {applied}")

    def _handle_open(self, request, response):
        del request
        position = self._float_param("open_gripper_position")
        max_effort = self._float_param("open_gripper_max_effort")
        velocity = self._float_param("open_gripper_velocity")
        force = self._float_param("open_gripper_force")
        success, message = self._send_gripper_goal(
            position,
            max_effort,
            velocity,
            force,
            "abrir gripper",
        )
        response.success = success
        response.message = message
        return response

    def _handle_close(self, request, response):
        del request
        position = self._float_param("close_gripper_position")
        max_effort = self._float_param("close_gripper_max_effort")
        velocity = self._float_param("close_gripper_velocity")
        force = self._float_param("close_gripper_force")
        success, message = self._send_gripper_goal(
            position,
            max_effort,
            velocity,
            force,
            "cerrar gripper",
        )
        response.success = success
        response.message = message
        return response

    def _wait_for_future_result(self, future, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)

        if not future.done():
            return None
        return future.result()

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _publish_scalar_command(self, publisher, value: float, label: str):
        msg = Float64MultiArray()
        msg.data = [float(value)]
        publisher.publish(msg)
        self.get_logger().info(f"{label}: data=[{value:.4f}]")

    def _send_gripper_goal(
        self,
        position: float,
        max_effort: float,
        velocity: float,
        force: float,
        label: str,
    ):
        with self.lock:
            if not self.gripper_client.wait_for_server(timeout_sec=1.0):
                return (
                    False,
                    f"Action del gripper '{self.gripper_action_name}' no esta disponible.",
                )

            goal_msg = GripperCommand.Goal()
            goal_msg.command.position = float(position)
            goal_msg.command.max_effort = float(max_effort)

            self._publish_scalar_command(
                self.velocity_pub,
                velocity,
                "Comando velocidad gripper",
            )
            self._publish_scalar_command(
                self.force_pub,
                force,
                "Comando fuerza gripper",
            )

            self.get_logger().info(
                f"Enviando {label}: position={position:.4f}, "
                f"max_effort={max_effort:.4f}, velocity={velocity:.4f}, force={force:.4f}"
            )
            goal_handle = self._wait_for_future_result(
                self.gripper_client.send_goal_async(goal_msg),
                timeout_sec=self.action_timeout_sec,
            )
            if goal_handle is None:
                return False, f"Timeout enviando goal para {label}."

            if not goal_handle.accepted:
                return False, f"Goal rechazado para {label}."

            result = self._wait_for_future_result(
                goal_handle.get_result_async(),
                timeout_sec=self.action_timeout_sec,
            )
            if result is None:
                return False, f"Timeout esperando resultado para {label}."

            self.get_logger().info(f"Comando completado: {label}.")
            return True, f"Comando completado: {label}."


def main(args=None):
    rclpy.init(args=args)
    node = Ur5GripperControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
