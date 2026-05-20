#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import (
    Int32MultiArray,
    Float64MultiArray,
)

from control_msgs.msg import DynamicJointState
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient


class FSRGraspDetector(Node):

    def __init__(self):

        super().__init__('fsr_grasp_detector')

        # -----------------------------
        # PARAMS
        # -----------------------------

        self.threshold = 300

        self.baseline_1 = None
        self.baseline_2 = None

        self.detected = False
        self.goal_handle = None
        self.gripper_joint_name = 'robotiq_hande_left_finger_joint'
        self.current_gripper_position = None
        self.detected_gripper_position = None
        self.close_velocity = 0.01
        self.close_force = 0.2
        self.hold_force = 0.2

        # -----------------------------
        # SUBSCRIBER
        # -----------------------------

        self.sub = self.create_subscription(
            Int32MultiArray,
            '/fsr',
            self.fsr_callback,
            10
        )

        self.dynamic_joint_sub = self.create_subscription(
            DynamicJointState,
            '/gripper/dynamic_joint_states',
            self.dynamic_joint_state_callback,
            10
        )

        # -----------------------------
        # VELOCITY PUB
        # -----------------------------

        self.vel_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper/gripper_velocity_controller/commands',
            10
        )

        # -----------------------------
        # FORCE PUB
        # -----------------------------

        self.force_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper/gripper_force_controller/commands',
            10
        )

        # -----------------------------
        # ACTION CLIENT
        # -----------------------------

        self.action_client = ActionClient(
            self,
            GripperCommand,
            '/gripper/gripper_action_controller/gripper_cmd'
        )

        # Esperar action server
        self.action_client.wait_for_server()

        self.get_logger().info('Nodo iniciado')

        # -----------------------------
        # SET LOW SPEED
        # -----------------------------

        vel_msg = Float64MultiArray()
        vel_msg.data = [self.close_velocity]

        self.vel_pub.publish(vel_msg)

        self.get_logger().info(
            f'Velocidad baja seteada: {self.close_velocity:.3f} m/s'
        )

        # -----------------------------
        # SET LOW FORCE
        # -----------------------------

        force_msg = Float64MultiArray()
        force_msg.data = [self.close_force]

        self.force_pub.publish(force_msg)

        self.get_logger().info(
            f'Fuerza baja seteada: {self.close_force:.2f}'
        )

        # -----------------------------
        # START CLOSING
        # -----------------------------

        self.close_gripper()

    # =====================================================

    def close_gripper(self):

        goal_msg = GripperCommand.Goal()

        goal_msg.command.position = 0.0
        goal_msg.command.max_effort = self.close_force

        goal_future = self.action_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self.close_goal_response_callback)

        self.get_logger().info(
            f'Cerrando gripper lentamente con fuerza {self.close_force:.2f}'
        )

    # =====================================================

    def close_goal_response_callback(self, future):

        self.goal_handle = future.result()

        if not self.goal_handle.accepted:

            self.get_logger().warn('Meta de cierre rechazada')
            self.goal_handle = None

    # =====================================================

    def dynamic_joint_state_callback(self, msg):

        if self.gripper_joint_name not in msg.joint_names:
            return

        joint_index = msg.joint_names.index(self.gripper_joint_name)
        interface_value = msg.interface_values[joint_index]

        if 'position' not in interface_value.interface_names:
            return

        position_index = interface_value.interface_names.index('position')
        self.current_gripper_position = interface_value.values[position_index] - 0.0025

    # =====================================================

    def stop_gripper(self):

        self.detected_gripper_position = self.current_gripper_position

        vel_msg = Float64MultiArray()
        vel_msg.data = [0.0]

        self.vel_pub.publish(vel_msg)

        if self.goal_handle is not None:

            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            self.goal_handle = None
            return

        self.hold_detected_position()

    # =====================================================

    def cancel_done_callback(self, future):

        self.hold_detected_position()

    # =====================================================

    def hold_detected_position(self):

        if self.detected_gripper_position is None:

            self.get_logger().warn(
                'No hay posicion de /gripper/dynamic_joint_states para mantener'
            )
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(self.detected_gripper_position)
        goal_msg.command.max_effort = self.hold_force

        hold_future = self.action_client.send_goal_async(goal_msg)
        hold_future.add_done_callback(self.hold_goal_response_callback)

        self.get_logger().info(
            f'Gripper detenido y posicion guardada: '
            f'{self.detected_gripper_position:.6f}, '
            f'fuerza {self.hold_force:.2f}'
        )

    # =====================================================

    def hold_goal_response_callback(self, future):

        hold_goal_handle = future.result()

        if not hold_goal_handle.accepted:

            self.get_logger().warn('Meta para mantener posicion rechazada')
            return

        self.get_logger().info('Gripper manteniendo posicion detectada')

    # =====================================================

    def fsr_callback(self, msg):

        if self.detected:
            return

        fsr1 = msg.data[0]
        fsr2 = msg.data[1]

        # Guardar baseline inicial
        if self.baseline_1 is None:

            self.baseline_1 = fsr1
            self.baseline_2 = fsr2

            self.get_logger().info(
                f'Baseline: {fsr1}, {fsr2}'
            )

            return

        # Cambio respecto baseline
        d1 = abs(fsr1 - self.baseline_1)
        d2 = abs(fsr2 - self.baseline_2)

        # Detectar contacto
        if d1 > self.threshold or d2 > self.threshold:

            self.detected = True

            self.get_logger().info(
                'OBJETO DETECTADO'
            )

            self.stop_gripper()


# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = FSRGraspDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
