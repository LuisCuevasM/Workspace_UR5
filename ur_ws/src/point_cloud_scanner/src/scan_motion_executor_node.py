#!/usr/bin/env python3

import copy
import math

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener


class ScanMotionExecutorNode(Node):
    def __init__(self):
        super().__init__("scan_motion_executor")

        self.declare_parameter("best_grasp_topic", "/graspgen/best_grasp")
        self.declare_parameter("execute_scan_service", "/point_cloud_scanner/execute_scan")
        self.declare_parameter("scan_dwell_event_topic", "/point_cloud_scanner/scan_dwell")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("tcp_link", "robotiq_hande_end")
        self.declare_parameter("group_name", "ur_arm")
        self.declare_parameter("cartesian_path_service", "/compute_cartesian_path")
        self.declare_parameter("execute_trajectory_action", "/execute_trajectory")
        self.declare_parameter("scan_radius", 0.2)
        self.declare_parameter("scan_start_angle_deg", 0.0)
        self.declare_parameter("scan_angle_deg", 180.0)
        self.declare_parameter("scan_direction", "right")
        self.declare_parameter("scan_height_offset", 0.0)
        self.declare_parameter("scan_waypoints", 6)
        self.declare_parameter("scan_dwell_sec", 2.0)
        self.declare_parameter("roll_offset_deg", 90.0)
        self.declare_parameter("look_at_object", True)
        self.declare_parameter("look_at_axis", "z")
        self.declare_parameter("cartesian_max_step", 0.01)
        self.declare_parameter("cartesian_jump_threshold", 0.0)
        self.declare_parameter("cartesian_min_fraction", 0.85)
        self.declare_parameter("max_velocity_scaling_factor", 0.03)
        self.declare_parameter("max_acceleration_scaling_factor", 0.03)
        self.declare_parameter("scan_dwell_joint_tolerance", 0.015)
        self.declare_parameter("scan_dwell_confirm_sec", 0.2)
        self.declare_parameter("scan_dwell_event_fallback_sec", 1.0)
        self.declare_parameter("avoid_collisions", True)

        self.best_grasp_topic = self.get_parameter("best_grasp_topic").value
        self.execute_scan_service = self.get_parameter("execute_scan_service").value
        self.scan_dwell_event_topic = self.get_parameter("scan_dwell_event_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.tcp_link = self.get_parameter("tcp_link").value
        self.group_name = self.get_parameter("group_name").value
        self.cartesian_path_service = self.get_parameter("cartesian_path_service").value
        self.execute_trajectory_action = self.get_parameter(
            "execute_trajectory_action"
        ).value

        self.latest_grasp = None
        self.latest_joint_state = None
        self.scan_active = False
        self.dwell_events = []
        self.execution_started_at = None

        self.cb_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cartesian_client = self.create_client(
            GetCartesianPath,
            self.cartesian_path_service,
            callback_group=self.cb_group,
        )
        self.execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            self.execute_trajectory_action,
            callback_group=self.cb_group,
        )
        self.dwell_event_pub = self.create_publisher(
            Int32,
            self.scan_dwell_event_topic,
            10,
        )

        self.create_subscription(
            PoseStamped,
            self.best_grasp_topic,
            self._on_grasp,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            50,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger,
            self.execute_scan_service,
            self._handle_execute_scan,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            f"Listo. Servicio de ejecucion: '{self.execute_scan_service}'. "
            f"Eventos de pausa en '{self.scan_dwell_event_topic}'."
        )

    def _on_grasp(self, msg):
        self.latest_grasp = msg

    def _on_joint_state(self, msg):
        self.latest_joint_state = msg
        self._publish_reached_dwell_events()

    def _handle_execute_scan(self, request, response):
        del request

        if self.scan_active:
            response.success = False
            response.message = "Ya hay un scan en ejecucion."
            return response
        if self.latest_grasp is None:
            response.success = False
            response.message = f"No hay pose recibida en {self.best_grasp_topic}."
            return response
        if self.latest_joint_state is None:
            response.success = False
            response.message = "No hay /joint_states para iniciar la planificacion."
            return response

        center = self._pose_in_target_frame(self.latest_grasp)
        tcp_pose = self._tcp_pose()
        if center is None or tcp_pose is None:
            response.success = False
            response.message = "No se pudo resolver TF para grasp/TCP."
            return response

        waypoints = self._build_scan_waypoints(center, tcp_pose)
        if not waypoints:
            response.success = False
            response.message = "No se generaron waypoints validos."
            return response

        self.scan_active = True
        self._request_cartesian_path(waypoints)
        response.success = True
        response.message = f"Scan solicitado con {len(waypoints)} waypoints."
        return response

    def _pose_in_target_frame(self, msg):
        try:
            if msg.header.frame_id and msg.header.frame_id != self.target_frame:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                )
                return do_transform_pose(msg.pose, transform)
            return msg.pose
        except Exception as exc:
            self.get_logger().error(
                f"No pude transformar grasp a '{self.target_frame}': {exc}"
            )
            return None

    def _tcp_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.tcp_link,
                rclpy.time.Time(),
            )
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            return pose
        except Exception as exc:
            self.get_logger().error(
                f"No pude leer TF del TCP '{self.tcp_link}' en '{self.target_frame}': {exc}"
            )
            return None

    def _build_scan_waypoints(self, center, tcp_pose):
        radius = float(self.get_parameter("scan_radius").value)
        angle_deg = float(self.get_parameter("scan_angle_deg").value)
        waypoint_count = max(3, int(self.get_parameter("scan_waypoints").value))
        if radius <= 0.0 or abs(angle_deg) <= 1.0e-6:
            self.get_logger().error("scan_radius y scan_angle_deg deben ser mayores que 0.")
            return []

        roll_offset = math.radians(float(self.get_parameter("roll_offset_deg").value))
        x_axis = self._rotate(self._roll((1.0, 0.0, 0.0), roll_offset), tcp_pose.orientation)
        y_axis = self._rotate(self._roll((0.0, 1.0, 0.0), roll_offset), tcp_pose.orientation)
        z_axis = self._rotate(self._roll((0.0, 0.0, 1.0), roll_offset), tcp_pose.orientation)

        start_angle = math.radians(float(self.get_parameter("scan_start_angle_deg").value))
        total_angle = math.radians(angle_deg)
        direction = self.get_parameter("scan_direction").value
        sign = -1.0 if direction == "right" else 1.0
        height_offset = float(self.get_parameter("scan_height_offset").value)

        waypoints = []
        for index in range(waypoint_count):
            t = index / float(waypoint_count - 1)
            angle = start_angle + sign * total_angle * t
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)

            waypoint = Pose()
            waypoint.position.x = center.position.x + x * x_axis[0] + y * y_axis[0] + height_offset * z_axis[0]
            waypoint.position.y = center.position.y + x * x_axis[1] + y * y_axis[1] + height_offset * z_axis[1]
            waypoint.position.z = center.position.z + x * x_axis[2] + y * y_axis[2] + height_offset * z_axis[2]
            if bool(self.get_parameter("look_at_object").value):
                waypoint.orientation = self._look_at_orientation(
                    waypoint,
                    center,
                    tcp_pose.orientation,
                )
            else:
                waypoint.orientation = tcp_pose.orientation
            waypoints.append(waypoint)

        return waypoints

    def _look_at_orientation(self, waypoint, center, reference_orientation):
        direction = self._normalize(
            (
                center.position.x - waypoint.position.x,
                center.position.y - waypoint.position.y,
                center.position.z - waypoint.position.z,
            )
        )
        if direction is None:
            return reference_orientation

        look_axis = self.get_parameter("look_at_axis").value
        if look_axis == "-z":
            z_axis = self._scale(direction, -1.0)
        else:
            z_axis = direction

        up_hint = self._rotate((0.0, 1.0, 0.0), reference_orientation)
        x_axis = self._normalize(self._cross(up_hint, z_axis))
        if x_axis is None:
            up_hint = self._rotate((1.0, 0.0, 0.0), reference_orientation)
            x_axis = self._normalize(self._cross(up_hint, z_axis))
        if x_axis is None:
            return reference_orientation

        y_axis = self._cross(z_axis, x_axis)
        return self._quaternion_from_axes(x_axis, y_axis, z_axis)

    def _request_cartesian_path(self, waypoints):
        if not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.scan_active = False
            self.get_logger().error(
                f"Servicio cartesiano '{self.cartesian_path_service}' no disponible."
            )
            return

        request = GetCartesianPath.Request()
        request.header.frame_id = self.target_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state = self._robot_state_from_joint_state(self.latest_joint_state)
        request.group_name = self.group_name
        request.link_name = self.tcp_link
        request.waypoints = waypoints
        request.max_step = float(self.get_parameter("cartesian_max_step").value)
        request.jump_threshold = float(self.get_parameter("cartesian_jump_threshold").value)
        request.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)

        self.get_logger().info(
            f"Solicitando path cartesiano con {len(waypoints)} waypoints para '{self.tcp_link}' "
            "(la velocidad se escala despues de calcular la trayectoria)."
        )
        future = self.cartesian_client.call_async(request)
        future.add_done_callback(self._on_cartesian_path)

    def _on_cartesian_path(self, future):
        try:
            response = future.result()
            min_fraction = float(self.get_parameter("cartesian_min_fraction").value)
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                self.scan_active = False
                self.get_logger().error(
                    f"GetCartesianPath fallo: error_code={response.error_code.val}, "
                    f"fraction={response.fraction:.3f}"
                )
                return
            if response.fraction < min_fraction:
                self.scan_active = False
                self.get_logger().error(
                    f"Path incompleto: fraction={response.fraction:.3f} < {min_fraction:.3f}"
                )
                return

            self.get_logger().info(
                f"Path cartesiano listo: fraction={response.fraction:.3f}. Ejecutando."
            )
            trajectory = self._scale_trajectory_speed(response.solution)
            trajectory = self._insert_scan_dwells(trajectory)
            self._execute_trajectory(trajectory)
        except Exception as exc:
            self.scan_active = False
            self.get_logger().error(f"Error procesando path cartesiano: {exc}")

    def _scale_trajectory_speed(self, trajectory):
        velocity_scale = float(self.get_parameter("max_velocity_scaling_factor").value)
        acceleration_scale = float(
            self.get_parameter("max_acceleration_scaling_factor").value
        )
        velocity_scale = max(0.001, min(1.0, velocity_scale))
        acceleration_scale = max(0.001, min(1.0, acceleration_scale))
        time_scale = 1.0 / velocity_scale

        for point in trajectory.joint_trajectory.points:
            total_ns = (
                point.time_from_start.sec * 1_000_000_000
                + point.time_from_start.nanosec
            )
            scaled_ns = int(total_ns * time_scale)
            point.time_from_start.sec = scaled_ns // 1_000_000_000
            point.time_from_start.nanosec = scaled_ns % 1_000_000_000
            point.velocities = [value * velocity_scale for value in point.velocities]
            point.accelerations = [
                value * acceleration_scale for value in point.accelerations
            ]

        self.get_logger().info(
            f"Trayectoria ralentizada: vel_scale={velocity_scale:.3f}, "
            f"acc_scale={acceleration_scale:.3f}, time_scale={time_scale:.1f}."
        )
        return trajectory

    def _insert_scan_dwells(self, trajectory):
        dwell_sec = max(0.0, float(self.get_parameter("scan_dwell_sec").value))
        waypoint_count = max(3, int(self.get_parameter("scan_waypoints").value))
        points = trajectory.joint_trajectory.points
        if dwell_sec <= 0.0 or len(points) < 2:
            self.dwell_events = []
            return trajectory

        dwell_ns = int(dwell_sec * 1_000_000_000)
        dwell_indices = {
            max(1, min(len(points) - 1, round((index + 1) * (len(points) - 1) / waypoint_count)))
            for index in range(waypoint_count)
        }

        new_points = []
        added_ns = 0
        joint_names = list(trajectory.joint_trajectory.joint_names)
        dwell_events = []
        dwell_number = 0
        for index, point in enumerate(points):
            point_copy = copy.deepcopy(point)
            original_ns = self._point_time_ns(point)
            self._set_point_time_ns(point_copy, original_ns + added_ns)

            if index in dwell_indices:
                self._zero_point_motion(point_copy)
                new_points.append(point_copy)
                dwell_events.append(
                    {
                        "index": dwell_number,
                        "time_ns": original_ns + added_ns,
                        "joint_names": joint_names,
                        "positions": list(point_copy.positions),
                        "reached_since": None,
                        "published": False,
                    }
                )
                dwell_number += 1
                added_ns += dwell_ns

                dwell_point = copy.deepcopy(point_copy)
                self._set_point_time_ns(dwell_point, original_ns + added_ns)
                self._zero_point_motion(dwell_point)
                new_points.append(dwell_point)
            else:
                new_points.append(point_copy)

        trajectory.joint_trajectory.points = new_points
        self.dwell_events = dwell_events
        self.get_logger().info(
            f"Pausas de escaneo insertadas: {len(dwell_indices)} vertices x "
            f"{dwell_sec:.1f}s."
        )
        return trajectory

    @staticmethod
    def _point_time_ns(point):
        return point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec

    @staticmethod
    def _set_point_time_ns(point, total_ns):
        point.time_from_start.sec = total_ns // 1_000_000_000
        point.time_from_start.nanosec = total_ns % 1_000_000_000

    @staticmethod
    def _zero_point_motion(point):
        if point.velocities:
            point.velocities = [0.0 for _ in point.velocities]
        if point.accelerations:
            point.accelerations = [0.0 for _ in point.accelerations]

    def _execute_trajectory(self, trajectory):
        if not self.execute_client.wait_for_server(timeout_sec=1.0):
            self.scan_active = False
            self.get_logger().error(
                f"Action '{self.execute_trajectory_action}' no disponible."
            )
            return

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        future = self.execute_client.send_goal_async(goal)
        future.add_done_callback(self._on_execute_response)

    def _on_execute_response(self, future):
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.scan_active = False
                self.execution_started_at = None
                self.get_logger().error("ExecuteTrajectory rechazo el goal.")
                return

            self.execution_started_at = self.get_clock().now()
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_execute_result)
        except Exception as exc:
            self.scan_active = False
            self.execution_started_at = None
            self.get_logger().error(f"Error enviando ExecuteTrajectory: {exc}")

    def _on_execute_result(self, future):
        try:
            result = future.result()
            self.scan_active = False
            self.execution_started_at = None
            if result is None:
                self.get_logger().error("ExecuteTrajectory no devolvio resultado.")
                return

            error_code = result.result.error_code.val
            if error_code != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(
                    f"Fallo ejecutando trayectoria: error_code={error_code}"
                )
                return

            self.get_logger().info("Trayectoria de scan ejecutada correctamente.")
        except Exception as exc:
            self.scan_active = False
            self.execution_started_at = None
            self.get_logger().error(f"Error procesando resultado de ejecucion: {exc}")

    def _publish_reached_dwell_events(self):
        if not self.scan_active or self.latest_joint_state is None:
            return

        now = self.get_clock().now()
        tolerance = max(
            0.0,
            float(self.get_parameter("scan_dwell_joint_tolerance").value),
        )
        confirm_duration = Duration(
            seconds=max(
                0.0,
                float(self.get_parameter("scan_dwell_confirm_sec").value),
            )
        )

        for event in self.dwell_events:
            if event["published"]:
                continue
            if self._dwell_event_timed_out(event, now):
                self.get_logger().warn(
                    f"No confirme por joint_states el punto {event['index'] + 1}; "
                    "publicando evento por tiempo de trayectoria."
                )
                self._publish_dwell_event(event["index"])
                event["published"] = True
                return
            if not self._joint_state_matches(event, tolerance):
                event["reached_since"] = None
                return
            if event["reached_since"] is None:
                event["reached_since"] = now
                return
            if now - event["reached_since"] < confirm_duration:
                return

            self._publish_dwell_event(event["index"])
            event["published"] = True
            return

    def _dwell_event_timed_out(self, event, now):
        if self.execution_started_at is None:
            return False
        fallback_sec = max(
            0.0,
            float(self.get_parameter("scan_dwell_event_fallback_sec").value),
        )
        elapsed = now - self.execution_started_at
        fallback_ns = int(fallback_sec * 1_000_000_000)
        return elapsed.nanoseconds >= event["time_ns"] + fallback_ns

    def _joint_state_matches(self, event, tolerance):
        joint_position_by_name = {
            name: position
            for name, position in zip(
                self.latest_joint_state.name,
                self.latest_joint_state.position,
            )
        }
        errors = []
        for name, target in zip(event["joint_names"], event["positions"]):
            current = joint_position_by_name.get(name)
            if current is None:
                return False
            errors.append(abs(current - target))
        return bool(errors) and max(errors) <= tolerance

    def _publish_dwell_event(self, dwell_index):
        msg = Int32()
        msg.data = int(dwell_index)
        self.dwell_event_pub.publish(msg)
        self.get_logger().info(
            f"Robot detenido en punto de escaneo {dwell_index + 1}; evento publicado."
        )

    @staticmethod
    def _robot_state_from_joint_state(joint_state):
        robot_state = RobotState()
        robot_state.joint_state = joint_state
        robot_state.is_diff = True
        return robot_state

    @staticmethod
    def _roll(vector, angle):
        vx, vy, vz = vector
        c = math.cos(angle)
        s = math.sin(angle)
        return (
            vx,
            c * vy - s * vz,
            s * vy + c * vz,
        )

    @staticmethod
    def _normalize(vector):
        x, y, z = vector
        norm = math.sqrt(x * x + y * y + z * z)
        if norm <= 1.0e-9:
            return None
        return (x / norm, y / norm, z / norm)

    @staticmethod
    def _scale(vector, scalar):
        return (vector[0] * scalar, vector[1] * scalar, vector[2] * scalar)

    @staticmethod
    def _cross(a, b):
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    @staticmethod
    def _quaternion_from_axes(x_axis, y_axis, z_axis):
        m00, m01, m02 = x_axis[0], y_axis[0], z_axis[0]
        m10, m11, m12 = x_axis[1], y_axis[1], z_axis[1]
        m20, m21, m22 = x_axis[2], y_axis[2], z_axis[2]
        trace = m00 + m11 + m22

        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (m21 - m12) / s
            qy = (m02 - m20) / s
            qz = (m10 - m01) / s
        elif m00 > m11 and m00 > m22:
            s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            qw = (m21 - m12) / s
            qx = 0.25 * s
            qy = (m01 + m10) / s
            qz = (m02 + m20) / s
        elif m11 > m22:
            s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            qw = (m02 - m20) / s
            qx = (m01 + m10) / s
            qy = 0.25 * s
            qz = (m12 + m21) / s
        else:
            s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            qw = (m10 - m01) / s
            qx = (m02 + m20) / s
            qy = (m12 + m21) / s
            qz = 0.25 * s

        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    @staticmethod
    def _rotate(vector, quaternion):
        qx, qy, qz, qw = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            return vector

        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
        vx, vy, vz = vector
        uvx, uvy, uvz = qy * vz - qz * vy, qz * vx - qx * vz, qx * vy - qy * vx
        uuvx = qy * uvz - qz * uvy
        uuvy = qz * uvx - qx * uvz
        uuvz = qx * uvy - qy * uvx

        return (
            vx + 2.0 * (qw * uvx + uuvx),
            vy + 2.0 * (qw * uvy + uuvy),
            vz + 2.0 * (qw * uvz + uuvz),
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScanMotionExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
