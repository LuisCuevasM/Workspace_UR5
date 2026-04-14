#!/usr/bin/env python3

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.msg import GoalStatusArray
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ReturnHomeOnSuccess(Node):
    def __init__(self):
        super().__init__("ur5_return_home_on_success")

        self.group_name = self.declare_parameter("group_name", "ur_arm").value
        self.planning_pipeline = self.declare_parameter(
            "planning_pipeline", "ompl" #ntfields
        ).value
        self.planner_id = self.declare_parameter("planner_id", "RRTConnectkConfigDefault").value
        self.status_topic = self.declare_parameter(
            "status_topic", "/execute_trajectory/_action/status"
        ).value
        self.move_action_name = self.declare_parameter(
            "move_action_name", "/move_action"
        ).value
        self.gripper_action_name = self.declare_parameter(
            "gripper_action_name",
            "gripper/gripper_action_controller/gripper_cmd",
        ).value
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
                [0.8901, -1.0472, -2.0245, -1.0297, 1.4835, 3.3335],
            ).value
        )
        self.allowed_planning_time = float(
            self.declare_parameter("allowed_planning_time", 10.0).value
        )
        self.goal_joint_tolerance = float(
            self.declare_parameter("goal_joint_tolerance", 1e-3).value
        )
        self.max_velocity_scaling_factor = float(
            self.declare_parameter("max_velocity_scaling_factor", 0.2).value
        )
        self.max_acceleration_scaling_factor = float(
            self.declare_parameter("max_acceleration_scaling_factor", 0.2).value
        )
        self.close_gripper_position = float(
            self.declare_parameter("close_gripper_position", 0.0).value
        )
        self.close_gripper_max_effort = float(
            self.declare_parameter("close_gripper_max_effort", 0.0).value
        )

        if len(self.arm_joint_names) != len(self.home_joint_positions):
            raise RuntimeError(
                "arm_joint_names y home_joint_positions deben tener el mismo largo"
            )

        self.latest_arm_joint_state = None
        self.seen_succeeded_goal_ids = set()
        self.home_goal_active = False
        self.gripper_goal_active = False

        self.cb_group = ReentrantCallbackGroup()

        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            self.move_action_name,
            callback_group=self.cb_group,
        )
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            self.gripper_action_name,
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
            GoalStatusArray,
            self.status_topic,
            self._on_status_array,
            20,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            "Nodo listo. Escucha SUCCEEDED, cierra gripper y vuelve a home "
            f"por {self.move_action_name} usando {self.status_topic} "
            f"(planning_pipeline='{self.planning_pipeline}', planner_id='{self.planner_id}')."
        )

    def _on_joint_states(self, msg: JointState):
        joint_map = {
            name: position for name, position in zip(msg.name, msg.position)
        }

        if not all(name in joint_map for name in self.arm_joint_names):
            return

        arm_state = JointState()
        arm_state.header = msg.header
        arm_state.name = list(self.arm_joint_names)
        arm_state.position = [joint_map[name] for name in self.arm_joint_names]
        self.latest_arm_joint_state = arm_state

    def _goal_id_key(self, goal_info) -> tuple:
        return tuple(goal_info.goal_id.uuid)

    def _on_status_array(self, msg: GoalStatusArray):
        for status in msg.status_list:
            if status.status != GoalStatus.STATUS_SUCCEEDED:
                continue

            goal_key = self._goal_id_key(status.goal_info)
            if goal_key in self.seen_succeeded_goal_ids:
                continue

            self.seen_succeeded_goal_ids.add(goal_key)

            if self.home_goal_active:
                self.home_goal_active = False
                self.get_logger().info("Movimiento a home completado.")
                continue

            self.get_logger().info(
                "Se detecto SUCCEEDED en MoveIt. Cerrando gripper y luego volviendo a home."
            )
            self._send_gripper_goal()

    def _robot_state_from_joint_state(self, joint_state: JointState) -> RobotState:
        robot_state = RobotState()
        robot_state.joint_state = JointState()
        robot_state.joint_state.header = joint_state.header
        robot_state.joint_state.name = list(joint_state.name)
        robot_state.joint_state.position = list(joint_state.position)
        robot_state.is_diff = False
        return robot_state

    def _build_home_constraints(self) -> Constraints:
        constraints = Constraints()
        for joint_name, joint_position in zip(
            self.arm_joint_names, self.home_joint_positions
        ):
            constraints.joint_constraints.append(
                JointConstraint(
                    joint_name=joint_name,
                    position=float(joint_position),
                    tolerance_above=self.goal_joint_tolerance,
                    tolerance_below=self.goal_joint_tolerance,
                    weight=1.0,
                )
            )
        return constraints

    def _send_gripper_goal(self):
        if self.gripper_goal_active:
            self.get_logger().warn("Ya hay un comando de gripper en curso.")
            return

        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Action del gripper no esta disponible.")
            self._send_home_goal()
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = self.close_gripper_position
        goal_msg.command.max_effort = self.close_gripper_max_effort

        self.gripper_goal_active = True
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._on_gripper_goal_response)

    def _on_gripper_goal_response(self, future):
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.gripper_goal_active = False
                self.get_logger().error("El gripper rechazo el goal de cierre.")
                self._send_home_goal()
                return

            self.get_logger().info("Goal de cierre del gripper aceptado.")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_gripper_goal_result)
        except Exception as e:
            self.gripper_goal_active = False
            self.get_logger().error(f"Error enviando goal al gripper: {e}")
            self._send_home_goal()

    def _on_gripper_goal_result(self, future):
        try:
            result_wrapper = future.result()
            self.gripper_goal_active = False
            if result_wrapper is None:
                self.get_logger().error("El gripper no devolvio resultado.")
            else:
                self.get_logger().info("Cierre del gripper completado.")
        except Exception as e:
            self.gripper_goal_active = False
            self.get_logger().error(f"Error procesando resultado del gripper: {e}")

        self._send_home_goal()

    def _send_home_goal(self):
        if self.home_goal_active:
            self.get_logger().warn("Ya hay un retorno a home en curso.")
            return

        if self.latest_arm_joint_state is None:
            self.get_logger().warn(
                "No hay /joint_states completos del brazo; no puedo volver a home."
            )
            return

        if not self.move_group_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("MoveGroup no esta disponible para volver a home.")
            return

        goal = MoveGroup.Goal()
        goal.planning_options.plan_only = False

        req = goal.request
        req.group_name = self.group_name
        req.pipeline_id = self.planning_pipeline
        req.planner_id = self.planner_id
        req.start_state = self._robot_state_from_joint_state(
            self.latest_arm_joint_state
        )
        req.allowed_planning_time = self.allowed_planning_time
        req.num_planning_attempts = 5
        req.max_velocity_scaling_factor = self.max_velocity_scaling_factor
        req.max_acceleration_scaling_factor = self.max_acceleration_scaling_factor
        req.goal_constraints.append(self._build_home_constraints())

        self.home_goal_active = True
        self.get_logger().info(
            f"Enviando retorno a home con pipeline='{self.planning_pipeline or 'default'}' "
            f"y planner_id='{self.planner_id or 'default'}'."
        )
        send_goal_future = self.move_group_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_home_goal_response)

    def _on_home_goal_response(self, future):
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.home_goal_active = False
                self.get_logger().error("MoveGroup rechazo el goal de retorno a home.")
                return

            self.get_logger().info("Goal de retorno a home aceptado.")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_home_goal_result)
        except Exception as e:
            self.home_goal_active = False
            self.get_logger().error(f"Error enviando retorno a home: {e}")

    def _on_home_goal_result(self, future):
        try:
            result_wrapper = future.result()
            if result_wrapper is None:
                self.home_goal_active = False
                self.get_logger().error("MoveGroup no devolvio resultado para home.")
                return

            error_code = result_wrapper.result.error_code.val
            if error_code != 1:
                self.home_goal_active = False
                self.get_logger().error(
                    f"Fallo el retorno a home. error_code={error_code}"
                )
        except Exception as e:
            self.home_goal_active = False
            self.get_logger().error(f"Error procesando resultado de home: {e}")


def main():
    rclpy.init()
    node = ReturnHomeOnSuccess()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
