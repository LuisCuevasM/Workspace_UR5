#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_srvs.srv import Trigger

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive


def quat_from_euler(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class MoveGroupPoseExecutorBuffered(Node):
    """
    Executor con BUFFER (FIFO):
      - Subscribes: /ur5/target_pose (PoseStamped)
      - Encola TODOS los puntos que llegan
      - Ejecuta 1 a la vez usando MoveIt action: /move_action (MoveGroup)
      - Publishes status: /ur5/exec_status (String)

    Servicios:
      /executor/clear_queue (Trigger)  -> vacía la cola
      /executor/pause       (SetBool)  -> pause/resume ejecución
      /executor/get_stats   (Trigger)  -> retorna stats en texto
    """

    def __init__(self):
        super().__init__("ur5_move_group_executor_buffered")

        # Action name
        self.declare_parameter("action_name", "/move_action")

        # group / links
        self.declare_parameter("planning_group", "ur_manipulator")
        self.declare_parameter("target_link", "tool0")

        # Topics
        self.declare_parameter("in_topic", "/ur5/target_pose")
        self.declare_parameter("status_topic", "/ur5/exec_status")

        # Buffer
        self.declare_parameter("max_queue", 500)  # límite de cola (seguridad)

        # Planning params
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("vel_scale", 0.05)
        self.declare_parameter("acc_scale", 0.05)

        # tolerancias (no demasiado estrictas)
        self.declare_parameter("pos_tol", 0.001)  # 1 cm
        self.declare_parameter("ori_tol", 0.01)  # ~6°

        # Approach
        self.declare_parameter("use_approach", False)
        self.declare_parameter("approach_dz", 0.10)

        self.action_name = self.get_parameter("action_name").value
        self.group = self.get_parameter("planning_group").value
        self.target_link = self.get_parameter("target_link").value

        in_topic = self.get_parameter("in_topic").value
        status_topic = self.get_parameter("status_topic").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.pose_sub = self.create_subscription(PoseStamped, in_topic, self._on_pose, qos)

        self.action_client = ActionClient(self, MoveGroup, self.action_name)

        # Cola FIFO
        self.max_queue = int(self.get_parameter("max_queue").value)
        self.queue = deque()  # cada elemento: PoseStamped

        # Estado ejecución
        self._busy = False
        self._paused = False
        self._current_pose: PoseStamped | None = None
        self._approach_stage = 0  # 0=none, 1=approach, 2=final

        # Stats
        self.total_received = 0
        self.total_executed = 0
        self.total_failed = 0

        # Servicios
        self.srv_clear = self.create_service(Trigger, "/executor/clear_queue", self._handle_clear)
        self.srv_stats = self.create_service(Trigger, "/executor/get_stats", self._handle_stats)

        self.get_logger().info(
            f"Executor BUFFER listo.\n"
            f"- Sub: {in_topic}\n"
            f"- Status: {status_topic}\n"
            f"- Action: {self.action_name}\n"
            f"- max_queue={self.max_queue}\n"
            f"Servicios:\n"
            f"  ros2 service call /executor/clear_queue std_srvs/srv/Trigger {{}}\n"
            f"  ros2 service call /executor/pause example_interfaces/srv/SetBool \"{{data: true}}\"  # pausa\n"
            f"  ros2 service call /executor/pause example_interfaces/srv/SetBool \"{{data: false}}\" # resume\n"
            f"  ros2 service call /executor/get_stats std_srvs/srv/Trigger {{}}"
        )

    def _pub_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _on_pose(self, msg: PoseStamped):
        """Encola TODAS las poses que llegan."""
        if len(self.queue) >= self.max_queue:
            self.get_logger().error(f"Queue full (max_queue={self.max_queue}). Dropping pose.")
            self._pub_status("QUEUE_FULL_DROP")
            return

        self.queue.append(msg)
        self.total_received += 1

        if self.total_received == 8:
            self._start_next()

    def _start_next(self):
        if self._paused:
            self._pub_status("PAUSED")
            return

        if self._busy:
            return

        if not self.queue:
            self._pub_status(f"IDLE recv={self.total_received} exec={self.total_executed} fail={self.total_failed}")
            return

        if not self.action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().error(f"{self.action_name} action server not available")
            self._pub_status("NO_SERVER")
            return

        self._current_pose = self.queue.popleft()
        self._busy = True

        use_approach = bool(self.get_parameter("use_approach").value)
        if use_approach:
            self._approach_stage = 1
            pose = self._make_approach_pose(self._current_pose)
            goal = self._build_goal_from_pose(pose)
            self._send_goal(goal, tag="APPROACH")
        else:
            self._approach_stage = 0
            goal = self._build_goal_from_pose(self._current_pose)
            self._send_goal(goal, tag="GO")

    def _make_approach_pose(self, pose: PoseStamped) -> PoseStamped:
        dz = float(self.get_parameter("approach_dz").value)
        p = PoseStamped()
        p.header = pose.header
        p.pose = pose.pose
        p.pose.position.z = pose.pose.position.z + dz
        return p

    def _build_goal_from_pose(self, pose: PoseStamped) -> MoveGroup.Goal:
        allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        vel_scale = float(self.get_parameter("vel_scale").value)
        acc_scale = float(self.get_parameter("acc_scale").value)
        pos_tol = float(self.get_parameter("pos_tol").value)
        ori_tol = float(self.get_parameter("ori_tol").value)

        # Constraints tipo RViz
        pos_c = PositionConstraint()
        pos_c.header.frame_id = pose.header.frame_id
        pos_c.link_name = self.target_link
        pos_c.weight = 1.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [pos_tol, pos_tol, pos_tol]
        pos_c.constraint_region.primitives.append(box)
        pos_c.constraint_region.primitive_poses.append(pose.pose)

        ori_c = OrientationConstraint()
        ori_c.header.frame_id = pose.header.frame_id
        ori_c.link_name = self.target_link
        ori_c.orientation = pose.pose.orientation
        ori_c.absolute_x_axis_tolerance = ori_tol
        ori_c.absolute_y_axis_tolerance = ori_tol
        ori_c.absolute_z_axis_tolerance = ori_tol
        ori_c.weight = 1.0

        c = Constraints()
        c.name = "pose_goal"
        c.position_constraints.append(pos_c)
        c.orientation_constraints.append(ori_c)

        req = MotionPlanRequest()
        req.group_name = self.group
        req.goal_constraints = [c]
        req.allowed_planning_time = allowed_planning_time
        req.max_velocity_scaling_factor = vel_scale
        req.max_acceleration_scaling_factor = acc_scale

        # usar estado actual del robot
        req.start_state.is_diff = True
        req.num_planning_attempts = 5

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.look_around = False
        return goal

    def _send_goal(self, goal: MoveGroup.Goal, tag: str = ""):
        qlen = len(self.queue)
        self.get_logger().info(
            f"Enviando goal ({tag}) | q_left={qlen} recv={self.total_received} exec={self.total_executed} fail={self.total_failed}"
        )
        self._pub_status(f"SENT_{tag} q_left={qlen}")

        fut = self.action_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._on_goal_response(f, tag))

    def _on_goal_response(self, future, tag: str):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveIt")
            self.total_failed += 1
            self._pub_status("REJECTED")
            self._busy = False
            self._approach_stage = 0
            self._current_pose = None
            self._start_next()
            return

        res_fut = goal_handle.get_result_async()
        res_fut.add_done_callback(lambda f: self._on_result(f, tag))

    def _on_result(self, future, tag: str):
        res = future.result().result
        err = res.error_code.val

        if err == 1:
            # SUCCESS
            if self._approach_stage == 1:
                # ahora manda el FINAL
                self._approach_stage = 2
                final_goal = self._build_goal_from_pose(self._current_pose)
                self._send_goal(final_goal, tag="FINAL")
                return

            self.total_executed += 1
            self._pub_status(f"DONE exec={self.total_executed}/{self.total_received} q={len(self.queue)}")
        else:
            self.total_failed += 1
            self.get_logger().error(f"❌ FAIL ({tag}) error_code={err}")
            self._pub_status(f"FAIL_{err} fail={self.total_failed} exec={self.total_executed} q={len(self.queue)}")

        # reset estado y sigue con el siguiente
        self._busy = False
        self._approach_stage = 0
        self._current_pose = None
        self._start_next()

    # --- Servicios ---
    def _handle_clear(self, request, response):
        n = len(self.queue)
        self.queue.clear()
        response.success = True
        response.message = f"Cleared queue (removed {n} poses)."
        self._pub_status(f"CLEARED removed={n}")
        self.get_logger().info(response.message)
        return response

    def _handle_pause(self, request, response):
        self._paused = bool(request.data)
        response.success = True
        response.message = f"paused={self._paused}"
        self.get_logger().info(response.message)
        self._pub_status("PAUSED" if self._paused else "RESUMED")

        # si reanudamos y estamos libres, continua
        if (not self._paused) and (not self._busy):
            self._start_next()

        return response

    def _handle_stats(self, request, response):
        response.success = True
        response.message = (
            f"recv={self.total_received} exec={self.total_executed} fail={self.total_failed} "
            f"q={len(self.queue)} busy={self._busy} paused={self._paused}"
        )
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = MoveGroupPoseExecutorBuffered()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
