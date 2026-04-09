#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy
from control_msgs.action import GripperCommand
from rclpy.duration import Duration

class GripperJoyControl(Node):
    def __init__(self):
        super().__init__('gripper_joy_control')

        # 1. Cliente de la Acción del Gripper
        self._action_client = ActionClient(
            self, 
            GripperCommand, 
            'gripper/gripper_action_controller/gripper_cmd'
        )

        # 2. Suscriptor al Joystick
        self.subscription = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10
        )

        # Variables de control y tiempo
        self.ultima_accion_tiempo = self.get_clock().now()
        self.espera = Duration(seconds=1.0)

        self.get_logger().info('Listo para controlar el gripper con el mando (ROS2 Jazzy)')

    def joy_callback(self, msg):
        ahora = self.get_clock().now()
        
        # Verificamos cooldown de 1 segundo
        if (ahora - self.ultima_accion_tiempo) < self.espera:
            return

        # Botón 0: Mandar la posición específica que pediste (Cerrar/Ajustar)
        if msg.buttons[0] == 1:
            self.get_logger().info('Enviando comando: Posición 0.025')
            self.send_gripper_goal(0.025)
            self.ultima_accion_tiempo = ahora

        # Botón 1: Abrir (Posición 0.0 o máxima apertura)
        elif msg.buttons[1] == 1:
            self.get_logger().info('Enviando comando: Abrir (0.0)')
            self.send_gripper_goal(0.0)
            self.ultima_accion_tiempo = ahora

    def send_gripper_goal(self, position):
        # Esperar a que el servidor esté listo
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('¡El servidor del Gripper no está disponible!')
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 0.0  # Como en tu ejemplo

        self.get_logger().info(f'Ejecutando acción hacia posición: {position}')
        
        # Enviamos el objetivo de forma asíncrona
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GripperJoyControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()