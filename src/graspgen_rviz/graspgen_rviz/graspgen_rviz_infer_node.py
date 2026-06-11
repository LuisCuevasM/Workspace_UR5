import os
# Reduce la fragmentacion de VRAM (ver hint de CUDA OOM). Debe definirse antes
# de importar torch para que el allocator lo tome.
os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

import gc
import time
import json
import traceback
from pathlib import Path
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, Float32, Float32MultiArray
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

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

def transform_xyz(points, transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    rotation_matrix = R.from_quat(
        [rotation.x, rotation.y, rotation.z, rotation.w]
    ).as_matrix()
    translation_vec = np.array(
        [translation.x, translation.y, translation.z],
        dtype=np.float32,
    )
    return (points @ rotation_matrix.T) + translation_vec


def save_xyz_ply(path, points):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        for x, y, z in points:
            f.write(f"{float(x)} {float(y)} {float(z)}\n")


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
        self.declare_parameter("scanned_object_topic", "/point_cloud_scanner/object_cloud_complete")
        self.declare_parameter("scene_topic", "/camera/camera/depth/color/points")
        self.declare_parameter("gripper_config", "")
        self.declare_parameter("grasp_threshold", 0.6)
        self.declare_parameter("num_grasps", 200)
        self.declare_parameter("collision_threshold", 0.002)
        self.declare_parameter("collision_filter_retries", 2)
        self.declare_parameter("collision_relaxation_factor", 0.5)
        self.declare_parameter("min_collision_threshold", 0.0005)
        self.declare_parameter("debug_save_clouds", True)
        self.declare_parameter("debug_output_dir", "/tmp/graspgen_collision_debug")
        self.declare_parameter("auto_infer", False)
        self.declare_parameter("infer_service_name", "/graspgen/run_inference")
        self.declare_parameter("infer_sam_service_name", "/graspgen/run_inference_sam")
        self.declare_parameter("release_model_service_name", "/graspgen/release_model")
        self.declare_parameter("load_model_service_name", "/graspgen/load_model")
        self.declare_parameter("top_grasp_count", 5)
        self.declare_parameter("grasp_radial_scale", 0.0)
        self.declare_parameter("publish_all_grasps", False)
        # ───────────────────────────────────────────────────────────────────────

        config = self.get_parameter("gripper_config").value

        if not os.path.exists(config):
            raise RuntimeError("gripper_config no existe")

        self.grasp_cfg = load_grasp_cfg(config)
        # El sampler (redes CUDA) se carga bajo demanda y es liberable con
        # /graspgen/release_model para dejar VRAM al VLM.
        self.sampler = None
        self.load_sampler()

        self.gripper_info = get_gripper_info(self.grasp_cfg.data.gripper_name)
        self.gripper_collision_mesh = self.gripper_info.collision_mesh
        self.get_logger().info(
            f"Gripper collision mesh='{self.grasp_cfg.data.gripper_name}', "
            f"vertices={len(self.gripper_collision_mesh.vertices)}, "
            f"bounds={self.gripper_collision_mesh.bounds.astype(float).tolist()}, "
            f"extents={self.gripper_collision_mesh.extents.astype(float).tolist()} m."
        )

        self.object_pc = None
        self.scanned_object_pc = None
        self.scene_pc = None
        self.scene_frame_id = None
        self.scene_cloud_stamp = None
        self.frame_id = None
        self.scanned_frame_id = None
        self.cloud_stamp = None
        self.scanned_cloud_stamp = None
        self.cached_best_grasp = None
        self.cached_best_score = None
        self.inference_busy = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers que siempre existen
        self.marker_pub = self.create_publisher(MarkerArray, "/graspgen/markers", 10)
        self.best_grasp_pub = self.create_publisher(PoseStamped, "/graspgen/best_grasp", 10)
        self.best_score_pub = self.create_publisher(Float32, "/graspgen/best_score", 10)
        self.top_grasps_pub = self.create_publisher(PoseArray, "/graspgen/top_grasps", 10)
        self.top_scores_pub = self.create_publisher(Float32MultiArray, "/graspgen/top_scores", 10)

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
            self.get_parameter("scanned_object_topic").value,
            self.scanned_object_callback,
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

        self.infer_sam_service = self.create_service(
            Trigger,
            self.get_parameter("infer_sam_service_name").value,
            self.handle_infer_sam_request,
        )

        self.release_model_service = self.create_service(
            Trigger,
            self.get_parameter("release_model_service_name").value,
            self.handle_release_model,
        )

        self.load_model_service = self.create_service(
            Trigger,
            self.get_parameter("load_model_service_name").value,
            self.handle_load_model,
        )

        self.timer = None
        if self.get_parameter("auto_infer").value:
            self.timer = self.create_timer(2.0, self.run_grasp)

        self.get_logger().info(
            "GraspGen collision node ready "
            f"(auto_infer={self.get_parameter('auto_infer').value}, "
            f"object_topic='{self.get_parameter('object_topic').value}', "
            f"scanned_object_topic='{self.get_parameter('scanned_object_topic').value}', "
            f"service='{self.get_parameter('infer_service_name').value}', "
            f"sam_service='{self.get_parameter('infer_sam_service_name').value}')"
        )

    def object_callback(self, msg):
        self.frame_id = msg.header.frame_id
        self.cloud_stamp = msg.header.stamp
        pc = pointcloud2_to_xyz(msg)
        if pc is not None:
            self.object_pc = pc

    def scanned_object_callback(self, msg):
        self.scanned_frame_id = msg.header.frame_id
        self.scanned_cloud_stamp = msg.header.stamp
        pc = pointcloud2_to_xyz(msg)
        if pc is not None:
            self.scanned_object_pc = pc

    def get_active_object_cloud(self, force_sam: bool = False):
        if not force_sam and self.scanned_object_pc is not None:
            return (
                self.scanned_object_pc,
                self.scanned_frame_id,
                self.scanned_cloud_stamp,
                "scanner",
            )

        return self.object_pc, self.frame_id, self.cloud_stamp, "sam"

    def scene_callback(self, msg):
        self.scene_frame_id = msg.header.frame_id
        self.scene_cloud_stamp = msg.header.stamp
        pc = pointcloud2_to_xyz(msg)
        if pc is not None:
            self.scene_pc = pc

    def _cloud_stats(self, label, points, frame_id):
        if points is None or len(points) == 0:
            raise ValueError(f"{label}: nube vacia")
        if not np.all(np.isfinite(points)):
            raise ValueError(f"{label}: contiene NaN o Inf")

        minimum = np.min(points, axis=0)
        maximum = np.max(points, axis=0)
        extent = maximum - minimum
        center = np.mean(points, axis=0)
        self.get_logger().info(
            f"{label}: frame='{frame_id}', points={len(points)}, "
            f"centroid={center.tolist()}, min={minimum.tolist()}, "
            f"max={maximum.tolist()}, extent={extent.tolist()} m."
        )
        max_extent = float(np.max(extent))
        if max_extent > 2.0:
            self.get_logger().warn(
                f"{label}: extent={max_extent:.3f} m sospechoso; revisa unidades o inclusion de fondo."
            )
        if max_extent < 0.001:
            self.get_logger().warn(
                f"{label}: extent={max_extent:.6f} m sospechoso; revisa unidades."
            )
        return center, extent

    def _scene_in_object_frame(self, scene_pc, object_frame):
        if not object_frame or not self.scene_frame_id:
            raise ValueError(
                f"Frame invalido: object_frame='{object_frame}', scene_frame='{self.scene_frame_id}'"
            )
        if self.scene_frame_id == object_frame:
            return scene_pc

        transform = self.tf_buffer.lookup_transform(
            object_frame,
            self.scene_frame_id,
            Time.from_msg(self.scene_cloud_stamp),
            timeout=Duration(seconds=0.3),
        )
        t = transform.transform.translation
        q = transform.transform.rotation
        self.get_logger().info(
            f"Transformando escena para colision: '{self.scene_frame_id}' -> '{object_frame}', "
            f"translation=({t.x:.4f}, {t.y:.4f}, {t.z:.4f}), "
            f"quaternion=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})."
        )
        return transform_xyz(scene_pc, transform)

    @staticmethod
    def _grasp_debug_record(index, grasp, score, collision_free):
        position, quaternion = mat4_to_pose(grasp)
        return {
            "index": int(index),
            "score": float(score),
            "position": position.astype(float).tolist(),
            "quaternion_xyzw": quaternion.astype(float).tolist(),
            "matrix": grasp.astype(float).tolist(),
            "collision_free": bool(collision_free),
        }

    def _save_collision_debug(self, object_pc, scene_pc, grasps, conf, mask, frame_id, threshold):
        if not bool(self.get_parameter("debug_save_clouds").value):
            return

        output_dir = Path(self.get_parameter("debug_output_dir").value)
        output_dir.mkdir(parents=True, exist_ok=True)
        save_xyz_ply(output_dir / "selected_object_cloud_before_filter.ply", object_pc)
        save_xyz_ply(output_dir / "selected_object_cloud_after_filter.ply", object_pc)
        save_xyz_ply(output_dir / "collision_scene_cloud.ply", scene_pc)

        records = [
            self._grasp_debug_record(index, grasp, score, collision_free)
            for index, (grasp, score, collision_free) in enumerate(
                zip(grasps, conf, mask), start=1
            )
        ]
        payload = {
            "frame_id": frame_id,
            "scene_source_frame_id": self.scene_frame_id,
            "collision_threshold": float(threshold),
            "total": len(records),
            "collision_free": int(np.sum(mask)),
            "colliding": int(len(mask) - np.sum(mask)),
            "grasps": records,
        }
        with (output_dir / "grasp_candidates_before_collision.json").open(
            "w", encoding="utf-8"
        ) as f:
            json.dump(payload, f, indent=2)
        with (output_dir / "grasp_candidates_after_collision.json").open(
            "w", encoding="utf-8"
        ) as f:
            json.dump(
                {
                    **payload,
                    "grasps": [record for record in records if record["collision_free"]],
                },
                f,
                indent=2,
            )
        self.get_logger().info(f"Debug de colision guardado en '{output_dir}'.")

    def load_sampler(self):
        """Carga el sampler GraspGen (redes CUDA) si no esta cargado."""
        if self.sampler is not None:
            return
        self.sampler = GraspGenSampler(self.grasp_cfg)
        self.get_logger().info("Sampler GraspGen cargado en GPU.")

    def handle_release_model(self, request, response):
        """Libera las redes GraspGen de la GPU para dejar VRAM al VLM."""
        del request
        if self.inference_busy:
            response.success = False
            response.message = "Inferencia en curso, no se puede liberar el modelo"
            return response
        self.sampler = None
        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
            torch.cuda.ipc_collect()
        self.get_logger().info(
            "Modelo GraspGen liberado de la GPU (/graspgen/release_model)."
        )
        response.success = True
        response.message = "GraspGen model released"
        return response

    def handle_load_model(self, request, response):
        """Recarga las redes GraspGen en la GPU si fueron liberadas."""
        del request
        try:
            self.load_sampler()
        except Exception as exc:
            response.success = False
            response.message = f"No se pudo cargar GraspGen: {exc}"
            self.get_logger().error(response.message)
            return response
        response.success = True
        response.message = "GraspGen model loaded"
        return response

    def _compute_and_publish_best_grasp(self, force_sam: bool = False):
        if self.sampler is None:
            # El modelo pudo haberse liberado para el ciclo VLM; recargarlo.
            try:
                self.load_sampler()
            except Exception as exc:
                return False, f"GraspGen no esta cargado y no se pudo recargar: {exc}"

        object_pc, frame_id, cloud_stamp, object_source = self.get_active_object_cloud(force_sam=force_sam)

        if object_pc is None or self.scene_pc is None:
            return False, "No hay pointcloud de objeto o escena disponible"

        try:
            scene_pc = self._scene_in_object_frame(self.scene_pc, frame_id)
            self._cloud_stats("object_cloud", object_pc, frame_id)
            self._cloud_stats("collision_scene", scene_pc, frame_id)
        except (ValueError, TransformException) as exc:
            return False, f"FAILURE_INVALID_COLLISION_INPUT: {exc}"

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
        collision_threshold = float(self.get_parameter("collision_threshold").value)
        retries = max(0, int(self.get_parameter("collision_filter_retries").value))
        relaxation_factor = float(self.get_parameter("collision_relaxation_factor").value)
        min_threshold = float(self.get_parameter("min_collision_threshold").value)
        collision_mask = None
        used_threshold = collision_threshold
        for attempt in range(retries + 1):
            used_threshold = max(min_threshold, collision_threshold * relaxation_factor ** attempt)
            collision_mask = filter_colliding_grasps(
                scene_pc=scene_centered,
                grasp_poses=grasps,
                gripper_collision_mesh=self.gripper_collision_mesh,
                collision_threshold=used_threshold,
            )
            free_count = int(np.sum(collision_mask))
            self.get_logger().info(
                f"Filtro colision intento {attempt + 1}/{retries + 1}: "
                f"threshold={used_threshold:.4f} m, total={len(grasps)}, "
                f"free={free_count}, colliding={len(grasps) - free_count}."
            )
            for index, (score, is_free) in enumerate(zip(conf, collision_mask), start=1):
                self.get_logger().debug(
                    f"Collision candidate={index}/{len(grasps)}, score={float(score):.4f}, "
                    f"threshold={used_threshold:.4f} m, result={'free' if is_free else 'collision'}."
                )
            if free_count > 0:
                break

        self._save_collision_debug(
            object_pc,
            scene_pc,
            grasps,
            conf,
            collision_mask,
            frame_id,
            used_threshold,
        )

        free = grasps[collision_mask]
        colliding = grasps[~collision_mask]
        free_conf = conf[collision_mask]

        # --------------------------------------------------
        # VOLVER AL FRAME ORIGINAL
        # --------------------------------------------------
        free[:, :3, 3] += pc_mean
        colliding[:, :3, 3] += pc_mean

        scale = float(self.get_parameter("grasp_radial_scale").value)
        free = scale_grasp_radially(free, pc_mean, scale)
        colliding = scale_grasp_radially(colliding, pc_mean, scale)

        if len(free) == 0:
            self.get_logger().warn(
                "FAILURE_NO_COLLISION_FREE_GRASP: no hay grasps libres despues "
                f"del filtro de colision; total={len(grasps)}, "
                f"final_threshold={used_threshold:.4f} m."
            )
            return False, (
                "FAILURE_NO_COLLISION_FREE_GRASP: "
                f"total={len(grasps)}, final_threshold={used_threshold:.4f} m"
            )

        top_grasp_count = max(1, int(self.get_parameter("top_grasp_count").value))
        sorted_idx = np.argsort(-free_conf)
        top_idx = sorted_idx[:top_grasp_count]
        top_free = free[top_idx].copy()
        top_free_conf = free_conf[top_idx].copy()

        best_grasp = top_free[0].copy()
        best_score = float(top_free_conf[0])
        best_source = object_source
        self.cached_best_grasp = best_grasp.copy()
        self.cached_best_score = best_score

        header = Header()
        header.stamp = cloud_stamp if cloud_stamp is not None else self.get_clock().now().to_msg()
        header.frame_id = frame_id

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

        top_scores_msg = Float32MultiArray()
        top_scores_msg.data = [float(score) for score in top_free_conf]
        self.top_scores_pub.publish(top_scores_msg)
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
            ok, msg = self._compute_and_publish_best_grasp(force_sam=False)
            response.success = ok
            response.message = msg
            return response
        except Exception as exc:
            self.get_logger().error(
                f"Error inesperado durante inferencia GraspGen: {exc}\n{traceback.format_exc()}"
            )
            response.success = False
            response.message = f"FAILURE_GRASPGEN_EXCEPTION: {exc}"
            return response
        finally:
            self.inference_busy = False

    def handle_infer_sam_request(self, request, response):
        """Inferencia forzando nube SAM2, ignorando la nube del scanner."""
        del request

        if self.inference_busy:
            response.success = False
            response.message = "Inferencia ya en curso"
            return response

        self.inference_busy = True
        try:
            ok, msg = self._compute_and_publish_best_grasp(force_sam=True)
            response.success = ok
            response.message = msg
            return response
        except Exception as exc:
            self.get_logger().error(
                f"Error inesperado durante inferencia GraspGen SAM2: {exc}\n{traceback.format_exc()}"
            )
            response.success = False
            response.message = f"FAILURE_GRASPGEN_EXCEPTION: {exc}"
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
