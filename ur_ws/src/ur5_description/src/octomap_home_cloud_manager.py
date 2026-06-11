#!/usr/bin/env python3

import copy
import time
import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener


class OctomapHomeCloudManager(Node):
    def __init__(self):
        super().__init__("octomap_home_cloud_manager")

        self.cb_group = ReentrantCallbackGroup()

        self.input_cloud_topic = self.declare_parameter(
            "input_cloud_topic", "/sam2/scene_cloud_no_object"
        ).value
        self.output_cloud_topic = self.declare_parameter(
            "output_cloud_topic", "/octomap/home_scene_cloud"
        ).value
        self.target_frame = self.declare_parameter(
            "target_frame", "base_link"
        ).value
        self.clear_service_name = self.declare_parameter(
            "clear_service_name", "/clear_octomap"
        ).value
        self.capture_service_name = self.declare_parameter(
            "capture_service_name", "/octomap_home_cloud_manager/capture_home_cloud"
        ).value
        self.home_joint_tolerance = float(
            self.declare_parameter("home_joint_tolerance", 0.08).value
        )
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 2.0).value
        )
        self.capture_timeout_sec = float(
            self.declare_parameter("capture_timeout_sec", 5.0).value
        )
        self.arm_joint_names = list(
            self.declare_parameter(
                "arm_joint_names",
                [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
            ).value
        )
        self.home_joint_positions = list(
            self.declare_parameter(
                "home_joint_positions",
                [0.872, -1.0995, -2.1118, -1.04719, 1.5009, 0.0],
            ).value
        )

        if len(self.arm_joint_names) != len(self.home_joint_positions):
            raise RuntimeError(
                "home_joint_positions y arm_joint_names deben tener el mismo largo"
            )

        self.latest_arm_joint_state = None
        self.robot_in_home = False
        self.waiting_for_fresh_cloud = False
        self.home_entered_ns = 0
        self.cached_home_cloud = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.clear_octomap_client = self.create_client(
            Empty,
            self.clear_service_name,
            callback_group=self.cb_group,
        )
        self.capture_home_cloud_service = self.create_service(
            Trigger,
            self.capture_service_name,
            self._handle_capture_home_cloud,
            callback_group=self.cb_group,
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            50,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            PointCloud2,
            self.input_cloud_topic,
            self._on_input_cloud,
            10,
            callback_group=self.cb_group,
        )

        timer_period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.publish_timer = self.create_timer(
            timer_period,
            self._publish_cached_cloud,
            callback_group=self.cb_group,
        )
        self.output_cloud_pub = self.create_publisher(
            PointCloud2, self.output_cloud_topic, 10
        )

        self.get_logger().info(
            "OctomapHomeCloudManager listo "
            f"(input='{self.input_cloud_topic}', output='{self.output_cloud_topic}', "
            f"target_frame='{self.target_frame}', clear_service='{self.clear_service_name}', "
            f"capture_service='{self.capture_service_name}', "
            f"publish_rate={self.publish_rate_hz:.2f} Hz)."
        )

    def _wait_for_future_result(self, future, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)

        if not future.done():
            return None
        return future.result()

    def _on_joint_states(self, msg: JointState):
        joint_map = {name: position for name, position in zip(msg.name, msg.position)}
        if not all(name in joint_map for name in self.arm_joint_names):
            return

        arm_positions = [joint_map[name] for name in self.arm_joint_names]
        self.latest_arm_joint_state = arm_positions

        was_in_home = self.robot_in_home
        self.robot_in_home = self._positions_match_home(arm_positions)

        if self.robot_in_home and not was_in_home:
            self._handle_home_entry()
        elif not self.robot_in_home and was_in_home:
            self.get_logger().info(
                "Robot salio de home. Manteniendo la ultima nube estatica para OctoMap."
            )

    def _positions_match_home(self, positions) -> bool:
        deltas = [
            abs(current - home)
            for current, home in zip(positions, self.home_joint_positions)
        ]
        max_delta = max(deltas) if deltas else float("inf")
        return max_delta <= self.home_joint_tolerance

    def _handle_home_entry(self):
        self.home_entered_ns = self.get_clock().now().nanoseconds
        self.waiting_for_fresh_cloud = True
        self.cached_home_cloud = None
        self.get_logger().info(
            "Robot entro a home. Limpiando OctoMap y esperando una nube fresca sin objeto."
        )
        self._clear_octomap()

    def _clear_octomap(self):
        if not self.clear_octomap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f"Servicio '{self.clear_service_name}' no disponible; no se pudo limpiar OctoMap."
            )
            return

        future = self.clear_octomap_client.call_async(Empty.Request())
        response = self._wait_for_future_result(future, timeout_sec=2.0)
        if response is None:
            self.get_logger().warn("Timeout limpiando OctoMap al entrar a home.")
            return

        self.get_logger().info("OctoMap limpiado al entrar a home.")

    def _request_fresh_home_cloud(self):
        self.home_entered_ns = self.get_clock().now().nanoseconds
        self.waiting_for_fresh_cloud = True
        self.cached_home_cloud = None

    def _handle_capture_home_cloud(self, request, response):
        del request

        if not self.robot_in_home:
            response.success = False
            response.message = "El robot no esta en home; no se puede capturar la nube."
            return response

        self.get_logger().info(
            "Solicitud recibida para capturar una nube fresca de home sin objeto."
        )
        self._request_fresh_home_cloud()

        deadline = time.monotonic() + self.capture_timeout_sec
        while time.monotonic() < deadline:
            if not self.waiting_for_fresh_cloud and self.cached_home_cloud is not None:
                response.success = True
                response.message = (
                    f"Nube de home capturada y cacheada en '{self.output_cloud_topic}'."
                )
                return response
            time.sleep(0.05)

        response.success = False
        response.message = (
            "Timeout esperando una nube fresca de home sin objeto."
        )
        self.get_logger().warn(response.message)
        return response

    def _msg_stamp_to_ns(self, stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _lookup_cloud_transform(self, source_frame: str, stamp):
        lookup_timeout = Duration(seconds=0.2)
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=lookup_timeout,
            )
        except TransformException:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time(),
                timeout=lookup_timeout,
            )

    def _transform_cloud_to_target(self, msg: PointCloud2):
        if msg.header.frame_id == self.target_frame:
            return copy.deepcopy(msg)

        try:
            transform = self._lookup_cloud_transform(msg.header.frame_id, msg.header.stamp)
        except TransformException as ex:
            self.get_logger().warn(
                f"No pude transformar nube de '{msg.header.frame_id}' a "
                f"'{self.target_frame}': {ex}"
            )
            return None

        points_struct = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        points = np.array(
            [[point[0], point[1], point[2]] for point in points_struct],
            dtype=np.float32,
        )
        if points.size == 0:
            self.get_logger().warn("La nube recibida no tiene puntos validos.")
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        qw = rotation.w
        qx = rotation.x
        qy = rotation.y
        qz = rotation.z

        rotation_matrix = np.array(
            [
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
            ],
            dtype=np.float32,
        )
        translation_vec = np.array(
            [translation.x, translation.y, translation.z], dtype=np.float32
        )
        transformed_points = (points @ rotation_matrix.T) + translation_vec

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.target_frame
        return point_cloud2.create_cloud_xyz32(header, transformed_points.tolist())

    def _on_input_cloud(self, msg: PointCloud2):
        if not self.robot_in_home or not self.waiting_for_fresh_cloud:
            return

        msg_stamp_ns = self._msg_stamp_to_ns(msg.header.stamp)
        if msg_stamp_ns > 0 and msg_stamp_ns < self.home_entered_ns:
            self.get_logger().info(
                "Ignorando nube sin objeto anterior a la llegada a home."
            )
            return

        transformed_cloud = self._transform_cloud_to_target(msg)
        if transformed_cloud is None:
            return

        self.cached_home_cloud = transformed_cloud
        self.waiting_for_fresh_cloud = False
        self.get_logger().debug(
            "Nube fresca sin objeto capturada en home. "
            f"Se republicara continuamente para OctoMap en '{self.target_frame}'."
        )

    def _publish_cached_cloud(self):
        if self.cached_home_cloud is None:
            return

        cloud_msg = copy.deepcopy(self.cached_home_cloud)
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.output_cloud_pub.publish(cloud_msg)


def main():
    rclpy.init()
    node = OctomapHomeCloudManager()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
