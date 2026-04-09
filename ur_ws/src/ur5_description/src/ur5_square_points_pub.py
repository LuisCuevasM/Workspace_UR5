#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class UR5PoseSequenceWithGripper(Node):
    def __init__(self):
        super().__init__("ur5_pose_sequence_with_gripper")

        # ---------------- ROS config ----------------
        self.pose_topic = "/ur5/target_pose"
        self.frame_id = "base"
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)

        # 1 pose cada 5 segundos (lento)
        self.rate_hz = 0.1
        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_loop)

        # ---------------- Gripper action client ----------------
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            "/gripper/gripper_action_controller/gripper_cmd"
        )

        # ---------------- Poses ----------------
        p1 = self.make_pose(
            0.2127355619632019, 0.21154259461111768, 0.20821945967072986,
            0.37857995359206736, 0.9244706480136188, 0.03560584088090003, 0.027630848605721302
        )

        p2 = self.make_pose(
            0.21274525298157373, 0.21154741120686738, 0.3818221451451701,
            0.378607744782987, 0.9244605197657855, 0.035583551549574974, 0.027617636463562887
        )

        p3 = self.make_pose(
            0.5275904357574129, -0.10982083050989808, p2.pose.position.z,
            0.3557557187425684, 0.9344962745311591, -0.012324543095940704, 0.0016392397077266178
        )

        p4 = self.make_pose(
            0.5276448364868845, -0.1098207600837041, 0.22827647227646175,
            0.35575638555090494, 0.9344958398339311, -0.012338482466944935, 0.0016374746546610358
        )

        self.path = [p1, p2, p3, p4]
        self.idx = 0
        self.waiting_for_gripper = False

        self.get_logger().info("Nodo iniciado: Poses + Gripper sincronizado")

    # ---------------------------------------------------------
    def make_pose(self, x, y, z, qx, qy, qz, qw):
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)
        return msg

    # ---------------------------------------------------------
    def send_gripper_goal(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Gripper action server no disponible")
            return

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 0.0

        self.waiting_for_gripper = True
        send_future = self.gripper_client.send_goal_async(goal)
        send_future.add_done_callback(self.gripper_goal_response_cb)

    def gripper_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rechazado")
            self.waiting_for_gripper = False
            return

        self.get_logger().info("Gripper goal aceptado")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_cb)

    def gripper_result_cb(self, future):
        result = future.result().result
        self.get_logger().info(
            f"Gripper terminado | position={result.position:.3f} | reached={result.reached_goal}"
        )
        self.waiting_for_gripper = False

    # ---------------------------------------------------------
    def timer_loop(self):
        if self.waiting_for_gripper:
            return  # espera a que termine el gripper

        msg = self.path[self.idx]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(msg)

        self.get_logger().info(
            f"Publicando punto {self.idx+1}/{len(self.path)}"
        )

        # ---- Acciones de gripper ----
        if self.idx == 0:
            self.get_logger().warn("→ Cerrando gripper")
            self.send_gripper_goal(0.0135)

        if self.idx == 2:
            self.get_logger().warn("→ Abriendo gripper (soltar)")
            self.send_gripper_goal(0.250)

        self.idx += 1
        if self.idx >= len(self.path):
            self.idx = 0  # loop 
# -------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = UR5PoseSequenceWithGripper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
