#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Pose
from tf2_ros import Buffer, TransformListener

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    RobotState,
    PlanningScene,
    PlanningSceneWorld,
    CollisionObject,
)
from shape_msgs.msg import SolidPrimitive


class CartesianZExecutor(Node):
    def __init__(self):
        super().__init__("cartesian_z_executor")

        self.base_frame = "base_link"
        self.ee_link = "tool0"
        self.group_name = "ur_arm"

        self.max_step = 0.005          # 5 mm entre waypoints interpolados
        self.jump_threshold = 0.0      # desactiva salto articular relativo
        self.min_fraction = 0.99       # exigimos casi trayectoria completa
        self.z_epsilon = 0.001         # 1 mm: si el cambio es menor, no mover
        self.avoid_collisions = True

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )

        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        self.create_subscription(
            PoseStamped, "/graspgen/best_grasp", self._on_pose, 10
        )

        self.table_timer = self.create_timer(2.0, self._publish_table)

        self.get_logger().info("CartesianZExecutor listo.")
        self.get_logger().info(
            f"base_frame={self.base_frame}, ee_link={self.ee_link}, group={self.group_name}"
        )

    def _publish_table(self):
        co = CollisionObject()
        co.id = "table"
        co.header.frame_id = self.base_frame
        co.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [1.0, 1.0, 0.05]
        co.primitives.append(box)

        p = Pose()
        p.position.z = -0.0255
        p.orientation.w = 1.0
        co.primitive_poses.append(p)

        world = PlanningSceneWorld()
        world.collision_objects.append(co)

        scene = PlanningScene()
        scene.world = world
        scene.is_diff = True

        self.scene_pub.publish(scene)
        self.table_timer.cancel()
        self.get_logger().info("Mesa publicada en planning_scene.")

    def _get_current_ee_pose(self) -> PoseStamped:
        t = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.ee_link,
            rclpy.time.Time()
        )

        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = t.transform.translation.x
        ps.pose.position.y = t.transform.translation.y
        ps.pose.position.z = t.transform.translation.z
        ps.pose.orientation = t.transform.rotation
        return ps

    def _transform_pose_to_base(self, msg: PoseStamped) -> PoseStamped:
        if msg.header.frame_id == self.base_frame:
            return msg

        t = self.tf_buffer.lookup_transform(
            self.base_frame,
            msg.header.frame_id,
            rclpy.time.Time()
        )

        out = PoseStamped()
        out.header.frame_id = self.base_frame
        out.header.stamp = self.get_clock().now().to_msg()

        # Transform manual para pose usando tf2 ya disponible en transform
        # posición
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z

        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w

        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z

        # rotación quaternion -> matriz
        r00 = 1 - 2 * (qy * qy + qz * qz)
        r01 = 2 * (qx * qy - qz * qw)
        r02 = 2 * (qx * qz + qy * qw)

        r10 = 2 * (qx * qy + qz * qw)
        r11 = 1 - 2 * (qx * qx + qz * qz)
        r12 = 2 * (qy * qz - qx * qw)

        r20 = 2 * (qx * qz - qy * qw)
        r21 = 2 * (qy * qz + qx * qw)
        r22 = 1 - 2 * (qx * qx + qy * qy)

        out.pose.position.x = r00 * px + r01 * py + r02 * pz + tx
        out.pose.position.y = r10 * px + r11 * py + r12 * pz + ty
        out.pose.position.z = r20 * px + r21 * py + r22 * pz + tz

        # orientación: q_out = q_tf * q_in
        ax, ay, az, aw = qx, qy, qz, qw
        bx = msg.pose.orientation.x
        by = msg.pose.orientation.y
        bz = msg.pose.orientation.z
        bw = msg.pose.orientation.w

        out.pose.orientation.x = aw * bx + ax * bw + ay * bz - az * by
        out.pose.orientation.y = aw * by - ax * bz + ay * bw + az * bx
        out.pose.orientation.z = aw * bz + ax * by - ay * bx + az * bw
        out.pose.orientation.w = aw * bw - ax * bx - ay * by - az * bz

        return out

    def _build_z_only_target(self, current_ps: PoseStamped, grasp_ps_base: PoseStamped) -> Pose:
        target = Pose()

        # Mantener XY y orientación actual; cambiar solo Z
        target.orientation = grasp_ps_base.pose.orientation

        target.orientation = current_ps.pose.orientation
        return target

    def _distance_xyz(self, a: Pose, b: Pose) -> float:
        dx = a.position.x - b.position.x
        dy = a.position.y - b.position.y
        dz = a.position.z - b.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _on_pose(self, msg: PoseStamped):
        try:
            if not self.cartesian_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("Servicio /compute_cartesian_path no disponible.")
                return

            if not self.execute_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("Acción /execute_trajectory no disponible.")
                return

            current_ps = self._get_current_ee_pose()
            grasp_ps_base = self._transform_pose_to_base(msg)
            target_pose = self._build_z_only_target(current_ps, grasp_ps_base)

            dz = target_pose.position.z - current_ps.pose.position.z

            self.get_logger().info(
                "Pose actual:\n"
                f"  pos=({current_ps.pose.position.x:.6f}, {current_ps.pose.position.y:.6f}, {current_ps.pose.position.z:.6f})\n"
                f"  quat=({current_ps.pose.orientation.x:.6f}, {current_ps.pose.orientation.y:.6f}, "
                f"{current_ps.pose.orientation.z:.6f}, {current_ps.pose.orientation.w:.6f})"
            )
            self.get_logger().info(
                "Objetivo cartesiano Z-only:\n"
                f"  pos=({target_pose.position.x:.6f}, {target_pose.position.y:.6f}, {target_pose.position.z:.6f})\n"
                f"  quat=({target_pose.orientation.x:.6f}, {target_pose.orientation.y:.6f}, "
                f"{target_pose.orientation.z:.6f}, {target_pose.orientation.w:.6f})\n"
                f"  dz={dz:.6f}"
            )

            if abs(dz) < self.z_epsilon:
                self.get_logger().info("Cambio en Z despreciable. No se ejecuta movimiento.")
                return

            req = GetCartesianPath.Request()
            req.header.frame_id = self.base_frame
            req.header.stamp = self.get_clock().now().to_msg()
            req.group_name = self.group_name
            req.link_name = self.ee_link

            # Muy importante: estado inicial actual
            req.start_state = RobotState()
            req.start_state.is_diff = True

            # Solo un waypoint final porque queremos línea recta entre pose actual y target
            req.waypoints.append(target_pose)

            req.max_step = self.max_step
            req.jump_threshold = self.jump_threshold
            req.avoid_collisions = self.avoid_collisions

            future = self.cartesian_client.call_async(req)
            future.add_done_callback(self._cartesian_response_callback)

        except Exception as e:
            self.get_logger().error(f"Error en callback: {e}")

    def _cartesian_response_callback(self, future):
        try:
            response = future.result()

            self.get_logger().info(
                f"Cartesian path fraction = {response.fraction:.4f}"
            )

            if response.fraction < self.min_fraction:
                self.get_logger().warn(
                    f"Trayectoria cartesiana incompleta ({response.fraction:.4f} < {self.min_fraction:.2f}). No se ejecuta."
                )
                return

            if len(response.solution.joint_trajectory.points) == 0:
                self.get_logger().warn("La trayectoria cartesiana vino vacía.")
                return

            goal = ExecuteTrajectory.Goal()
            goal.trajectory = response.solution

            send_future = self.execute_client.send_goal_async(goal)
            send_future.add_done_callback(self._execute_goal_response_callback)

            self.get_logger().info("Trayectoria cartesiana enviada para ejecución.")

        except Exception as e:
            self.get_logger().error(f"Error procesando respuesta cartesiana: {e}")

    def _execute_goal_response_callback(self, future):
        try:
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().warn("ExecuteTrajectory rechazó el goal.")
                return

            self.get_logger().info("ExecuteTrajectory aceptó el goal.")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._execute_result_callback)

        except Exception as e:
            self.get_logger().error(f"Error en respuesta de ejecución: {e}")

    def _execute_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(
                f"Ejecución finalizada. error_code={result.error_code.val}"
            )
        except Exception as e:
            self.get_logger().error(f"Error leyendo resultado de ejecución: {e}")


def main():
    rclpy.init()
    node = CartesianZExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()