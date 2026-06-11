#!/usr/bin/env python3
"""Wrapper cliente Python de las robot skills del UR5.

Permite invocar los skills expuestos por `ur5_skill_server` de forma sencilla:

    import rclpy
    from rclpy.node import Node
    from robot_skills import RobotSkills

    rclpy.init()
    node = Node("robot_skills_demo")
    robot = RobotSkills(node)

    robot.grasp()   # SAM select + GraspGen
    robot.pick()    # toma y levanta el objeto (queda sosteniendo)
    robot.place()   # deposita y vuelve a home

Cada metodo devuelve (success: bool, message: str). El spin del nodo se hace
internamente con rclpy.spin_until_future_complete, asi que no necesitas un executor
externo (pero si ya tienes uno en otro hilo, tambien funciona).
"""

from std_srvs.srv import Trigger


class RobotSkills:
    def __init__(self, node, ns: str = "/robot", timeout: float = 180.0):
        """
        :param node: un rclpy.node.Node ya inicializado.
        :param ns: namespace de los servicios de skills (default "/robot").
        :param timeout: timeout por defecto (segundos) para esperar cada skill.
        """
        self._node = node
        self._ns = ns.rstrip("/")
        self._timeout = float(timeout)
        self._clients = {}
        for skill in ("grasp", "pick", "place", "pick_place", "push", "handover"):
            self._clients[skill] = node.create_client(
                Trigger, f"{self._ns}/{skill}"
            )

    # ------------------------------------------------------------------
    def _call(self, skill: str, timeout: float = None):
        timeout = self._timeout if timeout is None else float(timeout)
        client = self._clients[skill]
        if not client.wait_for_service(timeout_sec=5.0):
            msg = f"Servicio '{self._ns}/{skill}' no disponible"
            self._node.get_logger().error(msg)
            return False, msg

        future = client.call_async(Trigger.Request())
        import rclpy

        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        if not future.done():
            msg = f"Timeout esperando skill '{skill}'"
            self._node.get_logger().error(msg)
            return False, msg

        response = future.result()
        if response is None:
            return False, f"Sin respuesta de skill '{skill}'"
        self._node.get_logger().info(
            f"Skill '{skill}': success={response.success} | {response.message}"
        )
        return response.success, response.message

    # ------------------------------------------------------------------
    # API de skills
    # ------------------------------------------------------------------
    def grasp(self, timeout: float = None):
        """Segmenta el objeto (SAM2) y genera los grasps. No mueve el brazo."""
        return self._call("grasp", timeout)

    def pick(self, timeout: float = None):
        """Toma el objeto: approach -> grasp -> cierra -> levanta. Queda sosteniendo."""
        return self._call("pick", timeout)

    def place(self, timeout: float = None):
        """Lleva el objeto a la pose de deposito, abre el gripper y vuelve a home."""
        return self._call("place", timeout)

    def pick_place(self, timeout: float = None):
        """Conveniencia: pick seguido de place."""
        return self._call("pick_place", timeout)

    def push(self, timeout: float = None):
        """(Por definir) Empuja el objeto."""
        return self._call("push", timeout)

    def handover(self, timeout: float = None):
        """(Por definir) Entrega el objeto a un humano."""
        return self._call("handover", timeout)


def main(args=None):
    """Demo minima: grasp -> pick -> place."""
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node("robot_skills_demo")
    robot = RobotSkills(node)
    try:
        for skill, fn in (("grasp", robot.grasp), ("pick", robot.pick), ("place", robot.place)):
            ok, message = fn()
            node.get_logger().info(f"{skill} -> {ok}: {message}")
            if not ok:
                node.get_logger().error(f"Secuencia abortada en {skill}.")
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
