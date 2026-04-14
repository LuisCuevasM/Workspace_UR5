#!/usr/bin/env python3

import json
import math
from typing import List, Optional

import requests
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class NTFieldsClient(Node):
    def __init__(self):
        super().__init__('ntfields_client')

        self.declare_parameter('planner_url', 'http://172.19.0.2:8888/plan')
        self.declare_parameter(
            'joint_names',
            [
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint',
            ],
        )
        self.declare_parameter(
            'controller_action',
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        self.declare_parameter('request_timeout', 60.0)
        self.declare_parameter('time_step', 0.2)

        self.planner_url = self.get_parameter('planner_url').value
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.controller_action = self.get_parameter('controller_action').value
        self.request_timeout = float(self.get_parameter('request_timeout').value)
        self.time_step = float(self.get_parameter('time_step').value)

        self.latest_joint_state: Optional[JointState] = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.controller_action
        )

        self.get_logger().info(f'planner_url: {self.planner_url}')
        self.get_logger().info(f'controller_action: {self.controller_action}')
        self.get_logger().info(f'joint_names: {self.joint_names}')

    def joint_state_cb(self, msg: JointState):
        self.latest_joint_state = msg

    def get_current_q(self) -> Optional[List[float]]:
        if self.latest_joint_state is None:
            self.get_logger().warn('No joint state received yet')
            return None

        name_to_pos = {
            name: pos for name, pos in zip(self.latest_joint_state.name, self.latest_joint_state.position)
        }

        try:
            q = [float(name_to_pos[name]) for name in self.joint_names]
            return q
        except KeyError as e:
            self.get_logger().error(f'Missing joint in /joint_states: {str(e)}')
            self.get_logger().error(f'Available joints: {list(name_to_pos.keys())}')
            return None

    def request_plan(self, q_start: List[float], q_goal: List[float]) -> Optional[dict]:
        payload = {
            'q_start': q_start,
            'q_goal': q_goal,
        }

        self.get_logger().info(f'Requesting plan to {self.planner_url}')
        self.get_logger().info(f'q_start: {q_start}')
        self.get_logger().info(f'q_goal : {q_goal}')

        try:
            response = requests.post(
                self.planner_url,
                json=payload,
                timeout=self.request_timeout
            )
            response.raise_for_status()
            data = response.json()
        except requests.RequestException as e:
            self.get_logger().error(f'Planner request failed: {e}')
            return None
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON from planner: {e}')
            return None

        if not data.get('success', False):
            self.get_logger().error(f'Planner returned failure: {data}')
            return None

        self.get_logger().info(
            f'Planner OK | num_steps={data.get("num_steps")} '
            f'| iterations={data.get("iterations")} '
            f'| planning_time={data.get("planning_time")}'
        )

        traj = data.get('trajectory', [])
        if len(traj) == 0:
            self.get_logger().error('Planner returned empty trajectory')
            return None

        self.get_logger().info(f'First point: {traj[0]}')
        self.get_logger().info(f'Last point : {traj[-1]}')

        return data

    def build_joint_trajectory(self, trajectory_points: List[List[float]]) -> JointTrajectory:
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        for i, q in enumerate(trajectory_points):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]

            # Opcional: velocidades en cero
            pt.velocities = [0.0] * len(self.joint_names)

            t = (i + 1) * self.time_step
            sec = int(math.floor(t))
            nanosec = int((t - sec) * 1e9)
            pt.time_from_start.sec = sec
            pt.time_from_start.nanosec = nanosec

            traj_msg.points.append(pt)

        return traj_msg

    def send_trajectory(self, traj_msg: JointTrajectory) -> bool:
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Action server not available: {self.controller_action}')
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj_msg

        self.get_logger().info(f'Sending trajectory with {len(traj_msg.points)} points')
        send_goal_future = self.traj_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if goal_handle is None:
            self.get_logger().error('Failed to send trajectory goal')
            return False

        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            return False

        self.get_logger().info('Trajectory goal accepted')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result is None:
            self.get_logger().error('No result returned by trajectory action')
            return False

        status = result.status
        self.get_logger().info(f'Goal finished with status: {status}')
        return True

    def plan_and_execute(self, q_goal: List[float]) -> bool:
        q_start = self.get_current_q()
        if q_start is None:
            return False

        data = self.request_plan(q_start, q_goal)
        if data is None:
            return False

        trajectory_points = data['trajectory']
        traj_msg = self.build_joint_trajectory(trajectory_points)
        return self.send_trajectory(traj_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NTFieldsClient()

    # Ejemplo de q_goal fijo para prueba
    q_goal = [-1.3, 0.4, 1.1, 0.5, -0.5, 0.0]

    # Espera breve para recibir /joint_states
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest_joint_state is not None:
            break

    ok = node.plan_and_execute(q_goal)

    if ok:
        node.get_logger().info('Plan and execution completed')
    else:
        node.get_logger().error('Plan and execution failed')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()