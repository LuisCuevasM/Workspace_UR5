#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class GripperJointFiller(Node):
    def __init__(self):
        super().__init__("gripper_joint_filler")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(0.05, self.publish_gripper_state)  # 20 Hz

    def publish_gripper_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            "robotiq_hande_left_finger_joint",
            "robotiq_hande_right_finger_joint",
        ]
        msg.position = [0.025, -0.025]   # abierto
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(GripperJointFiller())
    rclpy.shutdown()

if __name__ == "__main__":
    main()