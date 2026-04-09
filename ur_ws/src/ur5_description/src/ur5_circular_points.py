#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class ProbedWallCircle(Node):
    def __init__(self):
        super().__init__("probed_wall_circle_pub")

        # --- CONFIGURACIÓN DE LA PARED ---
        self.p1 = (-0.456, 0.248)
        self.p2 = (-0.251, 0.451)

        self.center_z = 0.65
        self.orientation = {
            'x': 0.304,
            'y': -0.628,
            'z': -0.642,
            'w': 0.317
        }

        # --- PARÁMETROS DEL CÍRCULO ---
        self.radius = 0.075      # 7.5 cm
        self.num_points = 60     # resolución del círculo

        # --- ROS ---
        self.topic = "/ur5/target_pose"
        self.frame_id = "base"
        self.pub = self.create_publisher(PoseStamped, self.topic, 10)
        self.rate_hz = 2.0       # velocidad del movimiento

        # Geometría
        self.center_x, self.center_y, _ = self.calculate_wall_geometry()

        # Trayectoria circular
        self.path = self.generate_circle()
        self.idx = 0

        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_loop)

    def calculate_wall_geometry(self):
        x1, y1 = self.p1
        x2, y2 = self.p2
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0, 0.0

    def generate_circle(self):
        points = []

        # Vector unitario de la pared
        dx = self.p2[0] - self.p1[0]
        dy = self.p2[1] - self.p1[1]
        mod = math.sqrt(dx**2 + dy**2)
        u_x = dx / mod
        u_y = dy / mod

        for i in range(self.num_points + 1):
            theta = 2.0 * math.pi * i / self.num_points

            # Coordenadas locales del círculo
            ly = self.radius * math.cos(theta)
            lz = self.radius * math.sin(theta)

            pose = PoseStamped()
            pose.header.frame_id = self.frame_id

            # Proyección sobre la pared
            pose.pose.position.x = self.center_x + ly * u_x
            pose.pose.position.y = self.center_y + ly * u_y
            pose.pose.position.z = self.center_z + lz

            pose.pose.orientation.x = self.orientation['x']
            pose.pose.orientation.y = self.orientation['y']
            pose.pose.orientation.z = self.orientation['z']
            pose.pose.orientation.w = self.orientation['w']

            points.append(pose)

        return points

    def timer_loop(self):
        if self.idx >= len(self.path):
            self.idx = 0  # loop continuo

        msg = self.path[self.idx]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

        self.get_logger().info(
            f"Publicando punto circular {self.idx + 1}/{len(self.path)}"
        )

        self.idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = ProbedWallCircle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
