#!/usr/bin/env python3
"""Servicios de percepcion para el orquestador LangGraph (via rosbridge).

Expone dos servicios del contrato `ROS2_INTERFACES.md`:

    /robot/get_camera_image  (ur5_manipulation/GetCameraImage)
        Captura el frame RGB actual de la RealSense y lo devuelve en JPEG base64.

    /robot/perceive          (ur5_manipulation/Perceive)
        Enriquece la salida semantica del VLM (box + puntos normalizados) con
        SAM2 + depth: produce mascara (mask_id), nube de puntos segmentada
        (publicada en /perception/<name>/points) y una pose aproximada
        (centroide + PCA) en el frame base del robot.

Integracion con el stack existente (todo via std_srvs/Trigger + topics):
    - Empuja el prompt del VLM a SAM2 por el topic String '/sam2/vlm_prompt'
      (coords en pixeles) y dispara la inferencia con '/sam2/apply_vlm_prompt'.
    - Lee la nube segmentada en '/sam2/object_cloud' (frame optico de la camara)
      y la transforma a 'base_link' con tf2.

El nodo es agnostico al numero de objetos: SAM2 procesa uno a la vez, asi que
`perceive` itera secuencialmente sobre `objects_json`.
"""

import base64
import json
import threading
import time

import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile

from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Pose, PoseStamped
from cv_bridge import CvBridge

import cv2

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

from ur5_manipulation.srv import GetCameraImage, Perceive


def _rotmat_to_quat(R):
    """Matriz de rotacion 3x3 -> cuaternion [qx, qy, qz, qw]."""
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([0.0, 0.0, 0.0, 1.0])


def _pca_quaternion(pts):
    """Orientacion principal (PCA) de una nube Nx3 -> cuaternion right-handed."""
    if pts.shape[0] < 3:
        return np.array([0.0, 0.0, 0.0, 1.0])
    centered = pts - pts.mean(axis=0)
    cov = np.cov(centered.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    # eigh devuelve ascendente; ordenar descendente (eje principal primero).
    order = np.argsort(eigvals)[::-1]
    R = eigvecs[:, order]
    # Forzar base right-handed (det = +1).
    if np.linalg.det(R) < 0:
        R[:, 2] = -R[:, 2]
    return _rotmat_to_quat(R)


class RobotPerceptionNode(Node):
    def __init__(self):
        super().__init__("robot_perception_node")
        self.cb_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        # --- Parametros ---
        self.rgb_topic = self.declare_parameter(
            "rgb_topic", "/camera/camera/color/image_raw"
        ).value
        self.cam_info_topic = self.declare_parameter(
            "cam_info_topic", "/camera/camera/aligned_depth_to_color/camera_info"
        ).value
        self.sam_cloud_topic = self.declare_parameter(
            "sam_cloud_topic", "/sam2/object_cloud"
        ).value
        self.sam_prompt_topic = self.declare_parameter(
            "sam_prompt_topic", "/sam2/vlm_prompt"
        ).value
        self.sam_apply_service = self.declare_parameter(
            "sam_apply_service", "/sam2/apply_vlm_prompt"
        ).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.jpeg_quality = int(self.declare_parameter("jpeg_quality", 90).value)
        self.cloud_wait_timeout = float(
            self.declare_parameter("cloud_wait_timeout_sec", 30.0).value
        )
        self.sam_service_wait = float(
            self.declare_parameter("sam_service_wait_sec", 5.0).value
        )

        # --- Estado de la camara ---
        self.latest_rgb_msg = None
        self.latest_info_msg = None
        self._cloud_lock = threading.Lock()
        self._latest_cloud = None
        self._cloud_event = threading.Event()
        self._mask_counter = 0
        self._cloud_pubs = {}

        # --- Suscriptores ---
        self.create_subscription(
            Image, self.rgb_topic, self._on_rgb, 10, callback_group=self.cb_group
        )
        self.create_subscription(
            CameraInfo,
            self.cam_info_topic,
            self._on_info,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            PointCloud2,
            self.sam_cloud_topic,
            self._on_cloud,
            10,
            callback_group=self.cb_group,
        )

        # --- Publicador de prompt a SAM2 ---
        self.prompt_pub = self.create_publisher(String, self.sam_prompt_topic, 10)

        # --- Cliente SAM2 ---
        self.sam_apply_client = self.create_client(
            Trigger, self.sam_apply_service, callback_group=self.cb_group
        )

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Servicios expuestos ---
        self.create_service(
            GetCameraImage,
            "/robot/get_camera_image",
            self._handle_get_camera_image,
            callback_group=self.cb_group,
        )
        self.create_service(
            Perceive,
            "/robot/perceive",
            self._handle_perceive,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            "robot_perception_node listo: /robot/get_camera_image, /robot/perceive"
        )

    # ------------------------------------------------------------------
    # Callbacks de sensores
    # ------------------------------------------------------------------
    def _on_rgb(self, msg):
        self.latest_rgb_msg = msg

    def _on_info(self, msg):
        self.latest_info_msg = msg

    def _on_cloud(self, msg):
        with self._cloud_lock:
            self._latest_cloud = msg
        self._cloud_event.set()

    @staticmethod
    def _stamp_to_seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    # ------------------------------------------------------------------
    # /robot/get_camera_image
    # ------------------------------------------------------------------
    def _handle_get_camera_image(self, request, response):
        del request
        if self.latest_rgb_msg is None:
            response.success = False
            response.detail = f"Sin imagen RGB en '{self.rgb_topic}'."
            self.get_logger().warn(response.detail)
            return response

        msg = self.latest_rgb_msg
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, buf = cv2.imencode(
                ".jpg", cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            )
            if not ok:
                raise RuntimeError("cv2.imencode JPEG fallo")
            response.image_base64 = base64.b64encode(buf.tobytes()).decode("ascii")
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.detail = f"No se pudo codificar la imagen: {exc}"
            self.get_logger().error(response.detail)
            return response

        response.success = True
        response.encoding = "jpeg"
        response.width = int(msg.width)
        response.height = int(msg.height)
        response.frame_id = msg.header.frame_id
        response.stamp = self._stamp_to_seconds(msg.header.stamp)
        response.detail = ""
        self.get_logger().info(
            f"get_camera_image: {response.width}x{response.height} "
            f"frame='{response.frame_id}' stamp={response.stamp:.3f}"
        )
        return response

    # ------------------------------------------------------------------
    # /robot/perceive
    # ------------------------------------------------------------------
    def _handle_perceive(self, request, response):
        try:
            objects = json.loads(request.objects_json) if request.objects_json else []
        except json.JSONDecodeError as exc:
            response.success = False
            response.objects_json = "[]"
            response.failure_type = "object_not_found"
            response.detail = f"objects_json invalido: {exc}"
            self.get_logger().error(response.detail)
            return response

        if self.latest_rgb_msg is None or self.latest_info_msg is None:
            response.success = False
            response.objects_json = "[]"
            response.failure_type = "object_not_found"
            response.detail = "Sin RGB/camera_info disponibles para denormalizar."
            self.get_logger().warn(response.detail)
            return response

        width = int(self.latest_rgb_msg.width)
        height = int(self.latest_rgb_msg.height)

        results = []
        any_ok = False
        details = []
        for obj in objects:
            res, info = self._perceive_one(obj, width, height)
            results.append(res)
            any_ok = any_ok or res["segmentation_ok"]
            if info:
                details.append(info)

        response.objects_json = json.dumps(results)
        response.success = any_ok
        response.failure_type = "" if any_ok else "segmentation_failed"
        response.detail = "; ".join(details)
        self.get_logger().info(
            f"perceive: {len(results)} objeto(s), success={any_ok}"
        )
        return response

    def _perceive_one(self, obj, width, height):
        name = str(obj.get("name", f"object_{self._mask_counter}"))
        failed = {
            "name": name,
            "segmentation_ok": False,
            "mask_id": "",
            "pose_estimate": [0.0] * 7,
            "cloud_topic": "",
            "num_cloud_points": 0,
        }

        box_norm = obj.get("object_box_xyxy_norm")
        if not box_norm or len(box_norm) != 4:
            return failed, f"{name}: box invalido"

        # Denormalizar a pixeles.
        box_px = [
            box_norm[0] * width,
            box_norm[1] * height,
            box_norm[2] * width,
            box_norm[3] * height,
        ]
        point_coords = []
        point_labels = []
        for p in obj.get("positive_points_xy_norm", []) or []:
            point_coords.append([p[0] * width, p[1] * height])
            point_labels.append(1)
        for p in obj.get("negative_points_xy_norm", []) or []:
            point_coords.append([p[0] * width, p[1] * height])
            point_labels.append(0)

        prompt = {
            "box_xyxy": box_px,
            "point_coords": point_coords,
            "point_labels": point_labels,
            "target_object": name,
            "selected_safe_part": obj.get("selected_safe_part", ""),
        }

        # Disparar SAM2: publicar prompt y esperar la nube fresca.
        self._cloud_event.clear()
        request_time = time.monotonic()
        self.prompt_pub.publish(String(data=json.dumps(prompt)))

        # Tambien disparar el servicio (idempotente; si SAM ya proceso el topic,
        # vuelve a aplicar el ultimo prompt). Best-effort.
        if self.sam_apply_client.wait_for_service(timeout_sec=self.sam_service_wait):
            future = self.sam_apply_client.call_async(Trigger.Request())
            self._spin_wait(future, timeout=self.cloud_wait_timeout)

        cloud = self._wait_for_cloud(request_time)
        if cloud is None:
            return failed, f"{name}: timeout esperando nube de SAM2"

        pts = self._read_cloud_points(cloud)
        if pts.shape[0] < 50:
            return failed, f"{name}: nube insuficiente ({pts.shape[0]} pts)"

        pose = self._estimate_pose(pts, cloud.header)
        if pose is None:
            return failed, f"{name}: no se pudo transformar a {self.base_frame}"

        self._mask_counter += 1
        mask_id = f"mask_{self._mask_counter:02d}"
        cloud_topic = self._republish_cloud(name, cloud)

        result = {
            "name": name,
            "segmentation_ok": True,
            "mask_id": mask_id,
            "pose_estimate": pose,
            "cloud_topic": cloud_topic,
            "num_cloud_points": int(pts.shape[0]),
        }
        return result, ""

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _wait_for_cloud(self, after_monotonic):
        deadline = time.monotonic() + self.cloud_wait_timeout
        while time.monotonic() < deadline:
            if self._cloud_event.wait(timeout=0.1):
                self._cloud_event.clear()
                with self._cloud_lock:
                    cloud = self._latest_cloud
                if cloud is not None:
                    return cloud
        return None

    @staticmethod
    def _read_cloud_points(cloud):
        gen = point_cloud2.read_points(
            cloud, field_names=("x", "y", "z"), skip_nans=True
        )
        pts = np.array([[p[0], p[1], p[2]] for p in gen], dtype=np.float64)
        if pts.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        return pts

    def _estimate_pose(self, pts, header):
        centroid = pts.mean(axis=0)
        quat = _pca_quaternion(pts)

        pose = Pose()
        pose.position.x = float(centroid[0])
        pose.position.y = float(centroid[1])
        pose.position.z = float(centroid[2])
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])

        cloud_frame = header.frame_id
        if not cloud_frame or cloud_frame == self.base_frame:
            return self._pose_to_list(pose)

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                cloud_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as exc:
            self.get_logger().warn(
                f"TF {cloud_frame}->{self.base_frame} fallo: {exc}"
            )
            return None

        transformed = do_transform_pose(pose, tf)
        return self._pose_to_list(transformed)

    @staticmethod
    def _pose_to_list(pose):
        return [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]

    def _republish_cloud(self, name, cloud):
        safe = "".join(c if c.isalnum() else "_" for c in name.lower()).strip("_")
        topic = f"/perception/{safe}/points"
        pub = self._cloud_pubs.get(topic)
        if pub is None:
            qos = QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            pub = self.create_publisher(PointCloud2, topic, qos)
            self._cloud_pubs[topic] = pub
        pub.publish(cloud)
        return topic

    def _spin_wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.result() if future.done() else None


def main(args=None):
    rclpy.init(args=args)
    node = RobotPerceptionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
