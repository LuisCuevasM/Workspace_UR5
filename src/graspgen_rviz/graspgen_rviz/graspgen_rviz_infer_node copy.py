import os
import time
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2

from scipy.spatial.transform import Rotation as R

# PointCloud2 helpers (ROS2)
from sensor_msgs_py import point_cloud2 as pc2

# GraspGen
import torch
from grasp_gen.grasp_server import GraspGenSampler, load_grasp_cfg

# --- FUNCIONES AUXILIARES ---
def mat4_to_pose(T: np.ndarray):
    """4x4 -> (pos xyz, quat xyzw)"""
    q = R.from_matrix(T[:3, :3]).as_quat()  # x,y,z,w
    p = T[:3, 3]
    return p.astype(float), q.astype(float)

def make_line_marker(header: Header, mid: int, a: np.ndarray, b: np.ndarray, rgba, width: float):
    m = Marker()
    m.header = header
    m.ns = "grasp_axes"
    m.id = mid
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.scale.x = width
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.lifetime.sec = 0  # persist
    from geometry_msgs.msg import Point
    p1 = Point(x=float(a[0]), y=float(a[1]), z=float(a[2]))
    p2 = Point(x=float(b[0]), y=float(b[1]), z=float(b[2]))
    m.points = [p1, p2]
    return m

def make_gripper_marker(header: Header, mid: int, T: np.ndarray, color=(0.0, 1.0, 0.0, 1.0)):
    """Dibuja una pinza paralela 3D (como la de la imagen de referencia)."""
    m = Marker()
    m.header = header
    m.ns = "grasp_gripper"
    m.id = mid
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.scale.x = 0.003  # Grosor de las líneas
    m.color.r, m.color.g, m.color.b, m.color.a = color
    m.lifetime.sec = 0
    
    from geometry_msgs.msg import Point
    def to_pt(vec): return Point(x=float(vec[0]), y=float(vec[1]), z=float(vec[2]))
    
    R_mat = T[:3, :3]
    p = T[:3, 3] # El centro de los dedos (punto de contacto)
    
    # Dimensiones aproximadas para un Hand-E / 2F
    depth = 0.05      # Largo de los dedos
    span = 0.06       # Apertura de la pinza
    
    # Calcular vértices
    base_center = p - R_mat[:, 2] * depth
    left_base = base_center - R_mat[:, 1] * (span/2)
    right_base = base_center + R_mat[:, 1] * (span/2)
    left_tip = left_base + R_mat[:, 2] * depth
    right_tip = right_base + R_mat[:, 2] * depth
    stem_start = base_center - R_mat[:, 2] * 0.05
    
    # Trazar las líneas que forman la pinza
    m.points = [
        to_pt(stem_start), to_pt(base_center),     # Tallo base
        to_pt(left_base), to_pt(right_base),       # Puente de la base
        to_pt(left_base), to_pt(left_tip),         # Dedo izquierdo
        to_pt(right_base), to_pt(right_tip)        # Dedo derecho
    ]
    return m


def pointcloud2_to_xyz_rgb(msg: PointCloud2, use_rgb: bool):
    """
    Convierte PointCloud2 a:
      xyz: (N,3) float32
      rgb: (N,3) uint8 o None
    """
    field_names = [f.name for f in msg.fields]
    has_rgb = ("rgb" in field_names) or ("rgba" in field_names)

    xyz = []
    rgb = [] if (use_rgb and has_rgb) else None

    if rgb is not None:
        rgb_field = "rgb" if "rgb" in field_names else "rgba"
        for p in pc2.read_points(msg, field_names=("x", "y", "z", rgb_field), skip_nans=True):
            x, y, z, c = p
            xyz.append((x, y, z))

            if isinstance(c, float):
                c = np.frombuffer(np.float32(c).tobytes(), dtype=np.uint32)[0]
            r_ = (c >> 16) & 0xFF
            g_ = (c >> 8) & 0xFF
            b_ = c & 0xFF
            rgb.append((r_, g_, b_))
    else:
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            xyz.append((x, y, z))

    xyz = np.asarray(xyz, dtype=np.float32)
    if rgb is None:
        return xyz, None
    rgb = np.asarray(rgb, dtype=np.uint8)
    return xyz, rgb

def voxel_downsample_np(pc: np.ndarray, voxel: float) -> np.ndarray:
    """Reduce la nube de puntos usando un filtro de cuadrícula (voxel grid)."""
    if pc.shape[0] == 0:
        return pc
    q = np.floor(pc / voxel).astype(np.int32)
    _, idx = np.unique(q, axis=0, return_index=True)
    return pc[idx]

# --- NODO DE ROS 2 ---
class GraspGenRvizInferNode(Node):
    def __init__(self):
        super().__init__("graspgen_rviz_infer_node")

        # --- Parameters ---
        self.declare_parameter("pc_topic", "/sam2/object_cloud")
        self.declare_parameter("frame_id", "camera_color_optical_frame")

        self.declare_parameter("gripper_config", "")
        self.declare_parameter("grasp_threshold", 0.8)
        self.declare_parameter("num_grasps", 200)
        self.declare_parameter("topk_num_grasps", -1)
        self.declare_parameter("best_k", 1)  # top1 si =1, topk si >1

        # RViz markers
        self.declare_parameter("topic_posearray", "/graspgen/poses")
        self.declare_parameter("topic_markers", "/graspgen/markers")
        self.declare_parameter("axis_len", 0.06)
        self.declare_parameter("axis_width", 0.004)

        self.declare_parameter("max_points", 8000)      # <- clave
        self.declare_parameter("voxel", 0.003)          # 3mm

        self.max_points = int(self.get_parameter("max_points").value)
        self.voxel = float(self.get_parameter("voxel").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)

        # --- Load config + model (once) ---
        gripper_config = self.get_parameter("gripper_config").value
        if not gripper_config or not os.path.exists(gripper_config):
            raise RuntimeError(f"gripper_config no existe: '{gripper_config}'")

        self.grasp_cfg = load_grasp_cfg(gripper_config)
        self.gripper_name = self.grasp_cfg.data.gripper_name
        self.sampler = GraspGenSampler(self.grasp_cfg)

        # --- Publishers ---
        self.frame_id = self.get_parameter("frame_id").value
        self.pose_pub = self.create_publisher(PoseArray, self.get_parameter("topic_posearray").value, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.get_parameter("topic_markers").value, 10)

        # --- Subscriber ---
        pc_topic = self.get_parameter("pc_topic").value
        self.sub = self.create_subscription(PointCloud2, pc_topic, self.on_pc, 10)

        self.get_logger().info(f"Subscribed to: {pc_topic}")
        self.get_logger().info("GraspGen sampler loaded OK.")

        self._last_run = 0.0  # simple throttle if needed

    def publish_grasps(self, grasps_T: np.ndarray, conf: np.ndarray):
        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id=self.frame_id)

        pa = PoseArray()
        pa.header = header
        ma = MarkerArray()

        # Limpiar marcadores fantasma anteriores
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        ma.markers.append(delete_marker)

        for i, T in enumerate(grasps_T):
            p, q = mat4_to_pose(T)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = float(p[0]), float(p[1]), float(p[2])
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
            pa.poses.append(pose)

            # Dibujar la pinza verde
            ma.markers.append(make_gripper_marker(header, i, T, color=(0.0, 1.0, 0.0, 1.0)))

        self.pose_pub.publish(pa)
        self.marker_pub.publish(ma)


    def on_pc(self, msg: PointCloud2):
        self._last_run = time.time()

        xyz, _ = pointcloud2_to_xyz_rgb(msg, use_rgb=False)
        if xyz.size < 50:
            return

        # =========================================================
        # 1. CENTRAR LA NUBE (Obligatorio para la Red Neuronal)
        # =========================================================
        pc_mean = xyz.mean(axis=0)
        xyz_centered = xyz - pc_mean

        grasp_threshold = float(self.get_parameter("grasp_threshold").value)
        num_grasps = int(self.get_parameter("num_grasps").value)
        topk_num_grasps = int(self.get_parameter("topk_num_grasps").value)
        best_k = int(self.get_parameter("best_k").value)

        t0 = time.time()
        
        # =========================================================
        # 2. INFERENCIA CON LA NUBE CENTRADA
        # =========================================================
        grasps_t, conf_t = GraspGenSampler.run_inference(
            xyz_centered, 
            self.sampler,
            grasp_threshold=grasp_threshold,
            num_grasps=num_grasps,
        )
        dt = time.time() - t0

        if len(grasps_t) == 0:
            self.get_logger().info(f"No grasps (thr={grasp_threshold}). dt={dt:.3f}s")
            return

        grasps = grasps_t.detach().cpu().numpy()
        conf = conf_t.detach().cpu().numpy()
        # Asegurarnos de que el factor de escala homogénea sea 1
        grasps[:, 3, 3] = 1.0

        # --- Elegir los mejores ---
        order = np.argsort(-conf) 
        if best_k > 0:
            order = order[:min(best_k, len(order))]
        grasps = grasps[order]
        conf = conf[order]

        # =========================================================
        # 3. DEVOLVER LOS AGARRES A LA POSICIÓN REAL
        # =========================================================
        # Sumamos las coordenadas originales (X, Y, Z) a la columna de traslación de la matriz 4x4
        grasps[:, :3, 3] += pc_mean.astype(np.float32)

        self.get_logger().info(
            f"Grasps={len(conf)} | Pos={grasps[0, :3, 3]} | dt={dt:.3f}s"
        )

        # Publicar los marcadores en RViz
        self.publish_grasps(grasps, conf)
        
def main():
    rclpy.init()
    node = GraspGenRvizInferNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()