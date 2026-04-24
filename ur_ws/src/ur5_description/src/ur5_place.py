#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class GraspPoseStore(Node):
    def __init__(self):
        super().__init__("grasp_pose_store")

        self.grasp_pose = None

        self.create_subscription(
            PoseStamped,
            "/graspgen/best_grasp",
            self._on_pose_received,
            10,
        )

    def _on_pose_received(self, msg: PoseStamped):
        self.grasp_pose = msg



def main(args=None):
    rclpy.init(args=args)
    node = GraspPoseStore()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
