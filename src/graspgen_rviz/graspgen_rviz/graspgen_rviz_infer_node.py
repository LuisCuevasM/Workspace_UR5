import os
import time
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, Float32
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2

from scipy.spatial.transform import Rotation as R

from sensor_msgs_py import point_cloud2 as pc2

import torch

from grasp_gen.grasp_server import GraspGenSampler, load_grasp_cfg
from grasp_gen.utils.point_cloud_utils import filter_colliding_grasps
from grasp_gen.robot import get_gripper_info


def mat4_to_pose(T):
    q = R.from_matrix(T[:3, :3]).as_quat()
    p = T[:3, 3]
    return p, q


def pointcloud2_to_xyz(msg):

    pts = []

    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        pts.append([p[0], p[1], p[2]])

    if len(pts) == 0:
        return None

    return np.array(pts, dtype=np.float32)


def make_gripper_marker(header, mid, T, color):

    from geometry_msgs.msg import Point

    m = Marker()
    m.header = header
    m.ns = "grasps"
    m.id = mid
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.scale.x = 0.003

    m.color.r = color[0]
    m.color.g = color[1]
    m.color.b = color[2]
    m.color.a = 1.0

    R_mat = T[:3, :3]
    p = T[:3, 3]

    depth = 0.05
    span = 0.06

    base = p - R_mat[:, 2] * depth

    left = base - R_mat[:, 1] * span / 2
    right = base + R_mat[:, 1] * span / 2

    left_tip = left + R_mat[:, 2] * depth
    right_tip = right + R_mat[:, 2] * depth

    stem = base - R_mat[:, 2] * 0.05

    def pt(v):
        return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))

    m.points = [
        pt(stem), pt(base),
        pt(left), pt(right),
        pt(left), pt(left_tip),
        pt(right), pt(right_tip)
    ]

    return m

def rotate_marker_frame(T, axis='z', degrees=90.0):
    T_rot = np.eye(4)

    if axis == 'x':
        T_rot[:3, :3] = R.from_euler('x', degrees, degrees=True).as_matrix()
    elif axis == 'y':
        T_rot[:3, :3] = R.from_euler('y', degrees, degrees=True).as_matrix()
    elif axis == 'z':
        T_rot[:3, :3] = R.from_euler('z', degrees, degrees=True).as_matrix()
    else:
        raise ValueError(f"axis inválido: {axis}")

    return T @ T_rot


def scale_grasp_radially(grasps, center, scale=1.1):
    for g in grasps:
        p = g[:3, 3]
        p_rel = p - center
        r = np.linalg.norm(p_rel)
        if r < 1e-6:
            continue
        direction = p_rel / r
        r_new = r * scale
        p_new = center + direction * r_new
        g[:3, 3] = p_new
    return grasps


class GraspGenCollisionNode(Node):

    def __init__(self):

        super().__init__("graspgen_collision_node")

        self.declare_parameter("object_topic", "/sam2/object_cloud")
        self.declare_parameter("scene_topic", "/camera/camera/depth/color/points")
        self.declare_parameter("gripper_config", "")
        self.declare_parameter("grasp_threshold", 0.6)
        self.declare_parameter("num_grasps", 200)
        self.declare_parameter("collision_threshold", 0.02)
        self.declare_parameter("auto_infer", False)
        self.declare_parameter("infer_service_name", "/graspgen/run_inference")
        self.declare_parameter("top_grasp_count", 5)
        # ── NUEVO ──────────────────────────────────────────────────────────────
        # True  → publica PoseArray + markers verde/rojo además del mejor
        # False → publica solo el mejor grasp (PoseStamped, score, marker azul)
        self.declare_parameter("publish_all_grasps", False)
        # ───────────────────────────────────────────────────────────────────────

        config = self.get_parameter("gripper_config").value

        if not os.path.exists(config):
            raise RuntimeError("gripper_config no existe")

        self.grasp_cfg = load_grasp_cfg(config)
        self.sampler = GraspGenSampler(self.grasp_cfg)

        self.gripper_info = get_gripper_info(self.grasp_cfg.data.gripper_name)
        self.gripper_collision_mesh = self.gripper_info.collision_mesh

        self.object_pc = None
        self.scene_pc = None
        self.frame_id = None
        self.cached_best_grasp = None
        self.cached_best_score = None
        self.inference_busy = False

        # Publishers que siempre existen
        self.marker_pub = self.create_publisher(MarkerArray, "/graspgen/markers", 10)
        self.best_grasp_pub = self.create_publisher(PoseStamped, "/graspgen/best_grasp", 10)
        self.best_score_pub = self.create_publisher(Float32, "/graspgen/best_score", 10)
        self.top_grasps_pub = self.create_publisher(PoseArray, "/graspgen/top_grasps", 10)

        # Publisher opcional (solo útil cuando publish_all_grasps=True)
        self.pose_pub = self.create_publisher(PoseArray, "/graspgen/poses", 10)

        self.create_subscription(
            PointCloud2,
            self.get_parameter("object_topic").value,
            self.object_callback,
            10
        )

        self.create_subscription(
            PointCloud2,
            self.get_parameter("scene_topic").value,
            self.scene_callback,
            10
        )

        self.infer_service = self.create_service(
            Trigger,
            self.get_parameter("infer_service_name").value,
            self.handle_infer_request,
        )

        self.timer = None
        if self.get_parameter("auto_infer").value:
            self.timer = self.create_timer(2.0, self.run_grasp)

        self.get_logger().info(
            "GraspGen collision node ready "
            f"(auto_infer={self.get_parameter('auto_infer').value}, "
            f"service='{self.get_parameter('infer_service_name').value}')"
        )

    def object_callback(self, msg):
        self.frame_id = msg.header.frame_id
        pc = pointcloud2_to_xyz(msg)
        if pc is not None:
            self.object_pc = pc

    def scene_callback(self, msg):
        pc = pointcloud2_to_xyz(msg)
        if pc is not None:
            self.scene_pc = pc

    def _compute_and_publish_best_grasp(self):
        if self.object_pc is None or self.scene_pc is None:
            return False, "No hay pointcloud de objeto o escena disponible"

        object_pc = self.object_pc
        scene_pc = self.scene_pc

        publish_all = self.get_parameter("publish_all_grasps").value

        # --------------------------------------------------
        # CENTRAR OBJETO
        # --------------------------------------------------
        pc_mean = np.mean(object_pc, axis=0)
        object_centered = object_pc - pc_mean
        scene_centered = scene_pc - pc_mean

        # --------------------------------------------------
        # REDUCIR ESCENA
        # --------------------------------------------------
        if scene_centered.shape[0] > 8000:
            idx = np.random.choice(scene_centered.shape[0], 8000, replace=False)
            scene_centered = scene_centered[idx]

        t0 = time.time()

        grasps_t, conf_t = GraspGenSampler.run_inference(
            object_centered,
            self.sampler,
            grasp_threshold=self.get_parameter("grasp_threshold").value,
            num_grasps=self.get_parameter("num_grasps").value,
        )

        if len(grasps_t) == 0:
            return False, "La inferencia no devolvio grasps candidatos"

        grasps = grasps_t.detach().cpu().numpy()
        conf = conf_t.detach().cpu().numpy()
        grasps[:, 3, 3] = 1

        # --------------------------------------------------
        # CHECK COLISIONES
        # --------------------------------------------------
        collision_mask = filter_colliding_grasps(
            scene_pc=scene_centered,
            grasp_poses=grasps,
            gripper_collision_mesh=self.gripper_collision_mesh,
            collision_threshold=self.get_parameter("collision_threshold").value
        )

        free = grasps[collision_mask]
        colliding = grasps[~collision_mask]
        free_conf = conf[collision_mask]

        # --------------------------------------------------
        # VOLVER AL FRAME ORIGINAL
        # --------------------------------------------------
        free[:, :3, 3] += pc_mean
        colliding[:, :3, 3] += pc_mean

        scale = 0.0
        free = scale_grasp_radially(free, pc_mean, scale)
        colliding = scale_grasp_radially(colliding, pc_mean, scale)

        if len(free) == 0:
            self.get_logger().warn("No hay grasps libres después del filtro de colisión")
            return False, "No hay grasps libres despues del filtro de colision"

        top_grasp_count = max(1, int(self.get_parameter("top_grasp_count").value))
        sorted_idx = np.argsort(-free_conf)
        top_idx = sorted_idx[:top_grasp_count]
        top_free = free[top_idx].copy()
        top_free_conf = free_conf[top_idx].copy()

        best_grasp = top_free[0].copy()
        best_score = float(top_free_conf[0])
        best_source = "current"
        self.cached_best_grasp = best_grasp.copy()
        self.cached_best_score = best_score

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        top_pose_array = PoseArray()
        top_pose_array.header = header
        for g in top_free:
            p, q = mat4_to_pose(g)
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            pose.position.z = float(p[2])
            pose.orientation.x = float(q[0])
            pose.orientation.y = float(q[1])
            pose.orientation.z = float(q[2])
            pose.orientation.w = float(q[3])
            top_pose_array.poses.append(pose)

        self.top_grasps_pub.publish(top_pose_array)

        # --------------------------------------------------
        # MEJOR GRASP → siempre se publica
        # --------------------------------------------------
        best_pose_msg = PoseStamped()
        best_pose_msg.header = header

        p_best, q_best = mat4_to_pose(best_grasp)
        best_pose_msg.pose.position.x = float(p_best[0])
        best_pose_msg.pose.position.y = float(p_best[1])
        best_pose_msg.pose.position.z = float(p_best[2])
        best_pose_msg.pose.orientation.x = float(q_best[0])
        best_pose_msg.pose.orientation.y = float(q_best[1])
        best_pose_msg.pose.orientation.z = float(q_best[2])
        best_pose_msg.pose.orientation.w = float(q_best[3])

        score_msg = Float32()
        score_msg.data = best_score

        self.best_grasp_pub.publish(best_pose_msg)
        self.best_score_pub.publish(score_msg)

        # --------------------------------------------------
        # MARKERS
        # --------------------------------------------------
        ma = MarkerArray()

        delete = Marker()
        delete.action = Marker.DELETEALL
        ma.markers.append(delete)

        # Marker azul del mejor grasp → siempre
        best_grasp_marker = rotate_marker_frame(best_grasp, axis='z', degrees=90.0)

        ma.markers.append(
            make_gripper_marker(header, 10000, best_grasp_marker, (0.0, 0.0, 1.0))
        )

        for rank, g in enumerate(top_free[1:], start=2):
            top_marker = rotate_marker_frame(g, axis='z', degrees=90.0)
            ma.markers.append(
                make_gripper_marker(
                    header,
                    10000 + rank,
                    top_marker,
                    (0.0, 0.8, 0.8),
                )
            )
        
        if publish_all:
            # Markers verdes (libres) y rojos (colisionando)
            mid = 0

            for g in free:
                ma.markers.append(
                    make_gripper_marker(header, mid, g, (0.0, 1.0, 0.0))
                )
                mid += 1

            for g in colliding[:20]:
                ma.markers.append(
                    make_gripper_marker(header, mid, g, (1.0, 0.0, 0.0))
                )
                mid += 1

            # PoseArray con todos los grasps libres
            pa = PoseArray()
            pa.header = header

            for g in free:
                p, q = mat4_to_pose(g)
                pose = Pose()
                pose.position.x = float(p[0])
                pose.position.y = float(p[1])
                pose.position.z = float(p[2])
                pose.orientation.x = float(q[0])
                pose.orientation.y = float(q[1])
                pose.orientation.z = float(q[2])
                pose.orientation.w = float(q[3])
                pa.poses.append(pose)

            self.pose_pub.publish(pa)

        self.marker_pub.publish(ma)

        self.get_logger().info(
            f"grasps free={len(free)} colliding={len(colliding)} "
            f"best_score={best_score:.3f} source={best_source} "
            f"top_published={len(top_free)} "
            f"publish_all={publish_all} "
            f"dt={time.time()-t0:.3f}s"
        )
        return True, (
            f"Grasp listo: free={len(free)} colliding={len(colliding)} "
            f"best_score={best_score:.3f} source={best_source} "
            f"top_published={len(top_free)}"
        )

    def run_grasp(self):
        if self.inference_busy:
            self.get_logger().debug("Inferencia ya en curso, se omite este disparo.")
            return

        self.inference_busy = True
        try:
            ok, msg = self._compute_and_publish_best_grasp()
            if not ok:
                self.get_logger().warn(msg)
        finally:
            self.inference_busy = False

    def handle_infer_request(self, request, response):
        del request

        if self.inference_busy:
            response.success = False
            response.message = "Inferencia ya en curso"
            return response

        self.inference_busy = True
        try:
            ok, msg = self._compute_and_publish_best_grasp()
            response.success = ok
            response.message = msg
            return response
        finally:
            self.inference_busy = False


def main():

    rclpy.init()

    node = GraspGenCollisionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
