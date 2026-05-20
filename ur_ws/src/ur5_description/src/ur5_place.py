#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener


class UR5PlaceNode(Node):
    def __init__(self):
        super().__init__("ur5_place")

        self.input_pose_topic = self.declare_parameter(
            "input_pose_topic", "/graspgen/best_grasp"
        ).value
        self.output_pose_topic = self.declare_parameter(
            "output_pose_topic", "/ur5/place_pose"
        ).value
        self.target_frame = self.declare_parameter(
            "target_frame", "base_link"
        ).value
        #q space move to pre place 

        self.pre_place = list(
            self.declare_parameter(
                "place_joint_positions",
                [1.5010, -2.4609, -0.6981, -1.5708, 1.5708, 0.8552],
            ).value
        )
        self.place_x = float(self.declare_parameter("place_x", -0.045).value)
        self.place_y = float(self.declare_parameter("place_y", -0.709).value)
        self.place_z = float(self.declare_parameter("place_z", 0.360).value)

        self.place_pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        transient_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            PoseStamped,
            self.input_pose_topic,
            self._on_pose_received,
            10,
        )
        self.place_pose_pub = self.create_publisher(
            PoseStamped,
            self.output_pose_topic,
            transient_qos,
        )

        self.get_logger().info(
            f"Resolviendo place desde '{self.input_pose_topic}' hacia "
            f"'{self.output_pose_topic}' en frame '{self.target_frame}'. "
            f"Posicion place fija=({self.place_x:.4f}, "
            f"{self.place_y:.4f}, {self.place_z:.4f}); "
            "orientacion tomada del grasp."
        )

    def _on_pose_received(self, msg: PoseStamped):
        try:
            if msg.header.frame_id and msg.header.frame_id != self.target_frame:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                )
                grasp_pose = do_transform_pose(msg.pose, transform)
            else:
                grasp_pose = msg.pose

            place_pose_msg = PoseStamped()
            place_pose_msg.header.stamp = self.get_clock().now().to_msg()
            place_pose_msg.header.frame_id = self.target_frame
            place_pose_msg.pose.position.x = self.place_x
            place_pose_msg.pose.position.y = self.place_y
            place_pose_msg.pose.position.z = self.place_z
            place_pose_msg.pose.orientation = grasp_pose.orientation

            self.place_pose = place_pose_msg
            self.place_pose_pub.publish(self.place_pose)
            self.get_logger().info(
                "Pose de place calculada y publicada: "
                f"frame='{self.place_pose.header.frame_id}' "
                f"pos=({self.place_pose.pose.position.x:.4f}, "
                f"{self.place_pose.pose.position.y:.4f}, "
                f"{self.place_pose.pose.position.z:.4f})"
            )
        except Exception as e:
            self.get_logger().error(f"No pude calcular/publicar la pose de place: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UR5PlaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
