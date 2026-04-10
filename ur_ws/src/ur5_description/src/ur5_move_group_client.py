#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    CollisionObject,
    PlanningScene,
    PlanningSceneWorld,
)
from shape_msgs.msg import SolidPrimitive


class MinimalGraspExecutor(Node):
    def __init__(self):
        super().__init__("minimal_grasp_executor")

        self.ee_link = "robotiq_hande_end"
        self.table_top_z = 0.0
        self.min_target_z = 0.03

        self.latest_joint_state = None
        self.arm_joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MoveIt action client
        self.action_client = ActionClient(self, MoveGroup, "/move_action")

        # Planning scene publisher
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        # Subscriber al grasp
        self.create_subscription(
            PoseStamped,
            "/graspgen/best_grasp",
            self._on_pose,
            10,
        )

        # Subscriber a joint states
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            10,
        )

        # Publicar mesa una vez
        self.table_timer = self.create_timer(2.0, self._publish_table)

        self.get_logger().info("Nodo minimalista listo.")

    def _on_joint_states(self, msg: JointState):
        if len(msg.name) > 0 and len(msg.position) > 0:
            self.latest_joint_state = msg

    def _build_arm_joint_state(self):
        if self.latest_joint_state is None:
            return None

        name_to_pos = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))

        if not all(j in name_to_pos for j in self.arm_joint_names):
            missing = [j for j in self.arm_joint_names if j not in name_to_pos]
            self.get_logger().warn(f"Faltan joints del brazo en /joint_states: {missing}")
            return None

        js = JointState()
        js.header = self.latest_joint_state.header
        js.name = list(self.arm_joint_names)
        js.position = [name_to_pos[j] for j in self.arm_joint_names]
        js.velocity = []
        js.effort = []
        return js

    def _publish_table(self):
        co = CollisionObject(id="table", operation=CollisionObject.ADD)
        co.header.frame_id = "base_link"

        co.primitives.append(
            SolidPrimitive(
                type=SolidPrimitive.BOX,
                dimensions=[1.0, 1.0, 0.05],
            )
        )

        p = Pose()
        p.position.x = 0.0
        p.position.y = 0.0
        p.position.z = -0.0255
        p.orientation.w = 1.0
        co.primitive_poses.append(p)

        world = PlanningSceneWorld()
        world.collision_objects.append(co)

        self.scene_pub.publish(
            PlanningScene(world=world, is_diff=True)
        )

        self.get_logger().info("Mesa publicada en planning_scene.")
        self.table_timer.cancel()

    def _is_target_reachable(self, target_pose: Pose) -> bool:
        radial_distance = (
            target_pose.position.x ** 2 + target_pose.position.y ** 2
        ) ** 0.5

        if target_pose.position.z < self.min_target_z:
            self.get_logger().warn(
                f"Objetivo descartado: z={target_pose.position.z:.3f} m demasiado bajo."
            )
            return False

        if radial_distance > 0.85:
            self.get_logger().warn(
                f"Objetivo descartado: distancia radial {radial_distance:.3f} m fuera de alcance."
            )
            return False

        return True

    def _on_pose(self, msg: PoseStamped):
        try:
            # Transformar target a base_link
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                msg.header.frame_id,
                rclpy.time.Time(),
            )

            target_pose = do_transform_pose(msg.pose, transform)

            self.get_logger().info(
                f"Target pose en base_link: "
                f"pos=({target_pose.position.x:.3f}, "
                f"{target_pose.position.y:.3f}, "
                f"{target_pose.position.z:.3f}) "
                f"quat=({target_pose.orientation.x:.3f}, "
                f"{target_pose.orientation.y:.3f}, "
                f"{target_pose.orientation.z:.3f}, "
                f"{target_pose.orientation.w:.3f})"
            )

            if not self._is_target_reachable(target_pose):
                return

            arm_js = self._build_arm_joint_state()
            if arm_js is None:
                self.get_logger().warn("No pude construir start_state válido para ur_arm.")
                return

            self.get_logger().info(
                "Start state ur_arm: " +
                ", ".join(
                    f"{name}={pos:.4f}" for name, pos in zip(arm_js.name, arm_js.position)
                )
            )

            # Crear goal MoveIt
            goal = MoveGroup.Goal()
            goal.planning_options.plan_only = True

            req = goal.request
            req.group_name = "ur_arm"
            req.pipeline_id = "ompl"
            req.planner_id = "RRTConnectkConfigDefault"
            req.allowed_planning_time = 10.0
            req.num_planning_attempts = 10
            req.max_velocity_scaling_factor = 0.1
            req.max_acceleration_scaling_factor = 0.1

            # Start state explícito y filtrado al brazo
            req.start_state.joint_state = arm_js

            # Position constraint
            pc = PositionConstraint()
            pc.header.frame_id = "base_link"
            pc.link_name = self.ee_link
            pc.weight = 1.0
            pc.constraint_region.primitives.append(
                SolidPrimitive(
                    type=SolidPrimitive.BOX,
                    dimensions=[0.01, 0.01, 0.01],
                )
            )
            pc.constraint_region.primitive_poses.append(target_pose)

            # Orientation constraint
            oc = OrientationConstraint()
            oc.header.frame_id = "base_link"
            oc.link_name = self.ee_link
            oc.orientation = target_pose.orientation
            oc.weight = 1.0
            oc.absolute_x_axis_tolerance = 0.15
            oc.absolute_y_axis_tolerance = 0.15
            oc.absolute_z_axis_tolerance = 0.15

            constraints = Constraints()
            constraints.position_constraints.append(pc)
            constraints.orientation_constraints.append(oc)
            req.goal_constraints.append(constraints)

            self.action_client.wait_for_server()
            self.action_client.send_goal_async(goal)

            self.get_logger().info(
                f"Plan solicitado (sin ejecución) para {self.ee_link}: "
                f"{target_pose.position.x:.3f}, "
                f"{target_pose.position.y:.3f}, "
                f"{target_pose.position.z:.3f}"
            )

        except Exception as e:
            self.get_logger().error(f"Error en _on_pose: {e}")

def main():
    rclpy.init()
    node = MinimalGraspExecutor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()