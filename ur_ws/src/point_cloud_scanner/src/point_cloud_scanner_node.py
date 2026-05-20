#!/usr/bin/env python3

import math

import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, Int32
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class PointCloudScannerNode(Node):
    def __init__(self):
        super().__init__("point_cloud_scanner")

        self.declare_parameter("best_grasp_topic", "/graspgen/best_grasp")
        self.declare_parameter("marker_topic", "/point_cloud_scanner/scan_marker")
        self.declare_parameter("input_cloud_topic", "/sam2/object_cloud")
        self.declare_parameter("complete_cloud_topic", "/point_cloud_scanner/object_cloud_complete")
        self.declare_parameter("capture_scan_service", "/point_cloud_scanner/capture_scan")
        self.declare_parameter("execute_scan_service", "/point_cloud_scanner/execute_scan")
        self.declare_parameter("infer_service_name", "/graspgen/run_inference")
        self.declare_parameter("auto_infer_after_scan", True)
        self.declare_parameter("auto_infer_delay_sec", 0.5)
        self.declare_parameter("scan_dwell_event_topic", "/point_cloud_scanner/scan_dwell")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("tcp_link", "robotiq_hande_end")
        self.declare_parameter("scan_radius", 0.2)
        self.declare_parameter("scan_start_angle_deg", 0.0)
        self.declare_parameter("scan_angle_deg", 180.0)
        self.declare_parameter("scan_direction", "right")
        self.declare_parameter("scan_height_offset", 0.0)
        self.declare_parameter("scan_waypoints", 6)
        self.declare_parameter("line_width", 0.01)
        self.declare_parameter("roll_offset_deg", 90.0)
        self.declare_parameter("capture_duration_sec", 12.0)
        self.declare_parameter("publish_period_sec", 0.5)
        self.declare_parameter("sam_mask_settle_sec", 1.0)
        self.declare_parameter("voxel_size", 0.003)
        self.declare_parameter("crop_to_object", True)
        self.declare_parameter("object_crop_radius", 0.35)
        self.declare_parameter("max_accumulated_points", 250000)
        # Registration parameters
        self.declare_parameter("use_registration", True)
        self.declare_parameter("registration_voxel_size", 0.005)
        self.declare_parameter("icp_distance_threshold", 0.01)
        self.declare_parameter("icp_max_iter", 50)
        self.declare_parameter("ransac_max_iter", 100000)
        self.declare_parameter("icp_fitness_threshold", 0.1)
        self.declare_parameter("min_scan_points", 100)

        self.best_grasp_topic = self.get_parameter("best_grasp_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.input_cloud_topic = self.get_parameter("input_cloud_topic").value
        self.complete_cloud_topic = self.get_parameter("complete_cloud_topic").value
        self.capture_scan_service = self.get_parameter("capture_scan_service").value
        self.execute_scan_service = self.get_parameter("execute_scan_service").value
        self.infer_service_name = self.get_parameter("infer_service_name").value
        self.scan_dwell_event_topic = self.get_parameter("scan_dwell_event_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.tcp_link = self.get_parameter("tcp_link").value

        self.cb_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.motion_client = self.create_client(
            Trigger,
            self.execute_scan_service,
            callback_group=self.cb_group,
        )
        self.infer_client = self.create_client(
            Trigger,
            self.infer_service_name,
            callback_group=self.cb_group,
        )

        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, marker_qos)
        self.complete_cloud_pub = self.create_publisher(
            PointCloud2,
            self.complete_cloud_topic,
            10,
        )
        self.latest_grasp = None
        self.object_center = None
        self.capture_active = False
        self.capture_until = None
        self.expected_captures = 0
        self.pending_capture_events = []
        self.latest_cloud_msg = None
        self.latest_cloud_frame = ""
        self.dwell_events_received = 0
        self.snapshot_attempts = 0
        # Accumulated model as a single Open3D PointCloud
        self.model_o3d = None
        self.model_scan_count = 0
        self.model_has_rgb = False
        self.auto_infer_timers = []

        self.create_subscription(
            PoseStamped,
            self.best_grasp_topic,
            self._on_grasp,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            PointCloud2,
            self.input_cloud_topic,
            self._on_cloud,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            Int32,
            self.scan_dwell_event_topic,
            self._on_scan_dwell_event,
            10,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger,
            self.capture_scan_service,
            self._handle_capture_scan,
            callback_group=self.cb_group,
        )
        self.publish_timer = self.create_timer(
            max(0.1, float(self.get_parameter("publish_period_sec").value)),
            self._publish_accumulated_if_ready,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            f"Dibujando trayectoria desde '{self.best_grasp_topic}' en '{self.marker_topic}'. "
            f"Escuchando nube en '{self.input_cloud_topic}', eventos en "
            f"'{self.scan_dwell_event_topic}' y publicando acumulado en "
            f"'{self.complete_cloud_topic}'. Servicio de captura: '{self.capture_scan_service}'. "
            f"Inferencia post-scan: '{self.infer_service_name}'."
        )

    def _on_grasp(self, msg):
        self.latest_grasp = msg
        center = self._pose_in_target_frame(msg)
        tcp_orientation = self._tcp_orientation()
        if center is None or tcp_orientation is None:
            return
        self.object_center = center

        marker = self._build_marker(center, tcp_orientation)
        if marker is not None:
            self.marker_pub.publish(marker)
            first = marker.points[0]
            last = marker.points[-1]
            self.get_logger().info(
                f"Marker publicado en '{self.marker_topic}' con {len(marker.points)} puntos. "
                f"p0=({first.x:.3f}, {first.y:.3f}, {first.z:.3f}), "
                f"pN=({last.x:.3f}, {last.y:.3f}, {last.z:.3f}), "
                f"frame='{marker.header.frame_id}'."
            )

    def _handle_capture_scan(self, request, response):
        del request

        if self.capture_active:
            response.success = False
            response.message = "Ya hay una captura de nube en ejecucion."
            return response
        if self.latest_grasp is None:
            response.success = False
            response.message = f"No hay pose recibida en {self.best_grasp_topic}."
            return response

        center = self._pose_in_target_frame(self.latest_grasp)
        if center is None:
            response.success = False
            response.message = "No se pudo resolver TF para la pose del objeto."
            return response

        self.object_center = center
        self._reset_accumulator()
        duration = max(0.5, float(self.get_parameter("capture_duration_sec").value))
        self.capture_until = self.get_clock().now() + Duration(seconds=duration)
        self.expected_captures = max(1, int(self.get_parameter("scan_waypoints").value))
        self.capture_active = True

        if self.motion_client.wait_for_service(timeout_sec=0.2):
            future = self.motion_client.call_async(Trigger.Request())
            future.add_done_callback(self._on_motion_started)
            response.message = (
                f"Captura por eventos iniciada: esperando {self.expected_captures} "
                f"fotos de nube durante {duration:.1f}s."
            )
        else:
            self.capture_active = False
            self.capture_until = None
            self.expected_captures = 0
            response.message = (
                f"El servicio de movimiento '{self.execute_scan_service}' no esta disponible."
            )
            response.success = False
            return response

        response.success = True
        return response

    def _on_motion_started(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.capture_active = False
            self.capture_until = None
            self.expected_captures = 0
            self.get_logger().error(f"No pude solicitar movimiento de scan: {exc}")
            return

        if result.success:
            self.get_logger().info(result.message)
        else:
            self.capture_active = False
            self.capture_until = None
            self.expected_captures = 0
            self.get_logger().error(f"Movimiento de scan no iniciado: {result.message}")

    def _on_cloud(self, msg):
        self.latest_cloud_msg = msg
        self.latest_cloud_frame = msg.header.frame_id

    def _on_scan_dwell_event(self, msg):
        if not self.capture_active:
            return
        self.dwell_events_received += 1
        settle_sec = max(0.0, float(self.get_parameter("sam_mask_settle_sec").value))
        capture_time = self.get_clock().now() + Duration(seconds=settle_sec)
        self.pending_capture_events.append((int(msg.data), capture_time))
        self.get_logger().info(
            f"Punto de escaneo {int(msg.data) + 1} recibido; "
            f"capturare nube despues de {settle_sec:.1f}s para estabilizar SAM."
        )

    def _capture_latest_cloud_snapshot(self, scan_index):
        self.snapshot_attempts += 1
        if self.latest_cloud_msg is None:
            self.get_logger().warn(
                f"No hay nube disponible para capturar en punto {scan_index + 1}."
            )
            return False

        transformed = self._cloud_to_target_arrays(self.latest_cloud_msg)
        if transformed is None:
            self.get_logger().warn(
                f"No pude transformar/leer la nube del punto {scan_index + 1}. "
                f"frame_entrada='{self.latest_cloud_msg.header.frame_id}'."
            )
            return False
        xyz, rgb = transformed
        if xyz.size == 0:
            self.get_logger().warn(
                f"La nube del punto {scan_index + 1} no tiene puntos validos antes del crop."
            )
            return False

        xyz, rgb = self._crop_object_points(xyz, rgb)
        if xyz.size == 0:
            self.get_logger().warn(
                f"La nube del punto {scan_index + 1} quedo vacia despues del crop."
            )
            return False

        min_pts = int(self.get_parameter("min_scan_points").value)
        if xyz.shape[0] < min_pts:
            self.get_logger().warn(
                f"Nube del punto {scan_index + 1} tiene {xyz.shape[0]} pts "
                f"(minimo {min_pts}). Omitiendo."
            )
            return False

        ok = self._register_and_fuse(xyz, rgb)
        if ok:
            n_model = len(self.model_o3d.points) if self.model_o3d is not None else 0
            self.get_logger().info(
                f"Scan {scan_index + 1}: {xyz.shape[0]} pts entrada → "
                f"modelo con {n_model} pts ({self.model_scan_count} scans fusionados)."
            )
        else:
            self.get_logger().warn(
                f"Registro del scan {scan_index + 1} fallo; descartando snapshot."
            )
        return ok

    def _publish_accumulated_if_ready(self):
        if not self.capture_active:
            return

        now = self.get_clock().now()
        ready_events = [
            event for event in self.pending_capture_events if now >= event[1]
        ]
        self.pending_capture_events = [
            event for event in self.pending_capture_events if now < event[1]
        ]
        for scan_index, _ in ready_events:
            self._capture_latest_cloud_snapshot(scan_index)

        expected_done = (
            self.expected_captures > 0
            and self.model_scan_count >= self.expected_captures
        )
        timed_out = self.capture_until is not None and now >= self.capture_until
        if not expected_done and not timed_out:
            return

        pending_count = len(self.pending_capture_events)
        self.capture_active = False
        self.capture_until = None
        self.expected_captures = 0
        self.pending_capture_events = []
        cloud = self._build_complete_cloud()
        if cloud is None:
            self.get_logger().warn(
                "Captura finalizada sin puntos validos. "
                f"eventos_recibidos={self.dwell_events_received}, "
                f"intentos_snapshot={self.snapshot_attempts}, "
                f"hay_nube_sam={self.latest_cloud_msg is not None}, "
                f"frame_ultima_nube='{self.latest_cloud_frame}', "
                f"pending_events={pending_count}."
            )
            return

        self.complete_cloud_pub.publish(cloud)
        self.get_logger().info(
            f"Nube completa publicada en '{self.complete_cloud_topic}' con "
            f"{cloud.width} puntos en frame '{cloud.header.frame_id}'."
        )
        self._schedule_inference_after_scan()

    def _schedule_inference_after_scan(self):
        if not bool(self.get_parameter("auto_infer_after_scan").value):
            return

        delay_sec = max(0.0, float(self.get_parameter("auto_infer_delay_sec").value))
        timer_holder = {}

        def timer_callback():
            timer = timer_holder["timer"]
            timer.cancel()
            if timer in self.auto_infer_timers:
                self.auto_infer_timers.remove(timer)
            self._request_inference_after_scan()

        timer_holder["timer"] = self.create_timer(
            max(0.01, delay_sec),
            timer_callback,
            callback_group=self.cb_group,
        )
        self.auto_infer_timers.append(timer_holder["timer"])
        self.get_logger().info(
            f"Inferencia automatica programada en {delay_sec:.2f}s."
        )

    def _request_inference_after_scan(self):
        if not self.infer_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn(
                f"No pude ejecutar inferencia automatica: servicio "
                f"'{self.infer_service_name}' no disponible."
            )
            return

        future = self.infer_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_auto_inference_done)

    def _on_auto_inference_done(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Inferencia automatica fallo: {exc}")
            return

        if result.success:
            self.get_logger().info(
                f"Inferencia automatica terminada: {result.message}"
            )
        else:
            self.get_logger().warn(
                f"Inferencia automatica sin grasp valido: {result.message}"
            )

    def _reset_accumulator(self):
        self.model_o3d = None
        self.model_scan_count = 0
        self.model_has_rgb = False
        self.pending_capture_events = []
        self.dwell_events_received = 0
        self.snapshot_attempts = 0

    # ------------------------------------------------------------------ #
    # Registration pipeline                                                #
    # ------------------------------------------------------------------ #

    def _register_and_fuse(self, xyz, rgb):
        """Register a new scan into the accumulated model using FPFH+RANSAC+ICP."""
        source_full = self._to_o3d(xyz, rgb)
        voxel_final = float(self.get_parameter("voxel_size").value)

        if self.model_o3d is None:
            # First scan: downsample + outlier removal → initial model
            init = source_full.voxel_down_sample(voxel_final) if voxel_final > 0 else source_full
            init, _ = init.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            self.model_o3d = init
            self.model_scan_count = 1
            self.model_has_rgb = source_full.has_colors()
            return True

        if not bool(self.get_parameter("use_registration").value):
            src = source_full.voxel_down_sample(voxel_final) if voxel_final > 0 else source_full
            self.model_o3d += src
            self._trim_model()
            self.model_scan_count += 1
            return True

        voxel_reg = float(self.get_parameter("registration_voxel_size").value)
        try:
            T = self._full_registration(source_full, voxel_reg)
        except Exception as exc:
            self.get_logger().error(f"Registro fallo: {exc}")
            return False

        source_full.transform(T)
        src = source_full.voxel_down_sample(voxel_final) if voxel_final > 0 else source_full
        self.model_o3d += src
        self._trim_model()
        self.model_scan_count += 1
        return True

    def _full_registration(self, source_full, voxel_reg):
        """FPFH + RANSAC global registration followed by point-to-plane ICP."""
        src_down = self._preprocess_for_registration(source_full, voxel_reg)
        tgt_down = self._preprocess_for_registration(self.model_o3d, voxel_reg)

        src_fpfh = self._compute_fpfh(src_down, voxel_reg)
        tgt_fpfh = self._compute_fpfh(tgt_down, voxel_reg)

        ransac_result = self._ransac_global_registration(
            src_down, tgt_down, src_fpfh, tgt_fpfh, voxel_reg
        )

        icp_dist = float(self.get_parameter("icp_distance_threshold").value)
        icp_result = self._icp_refine(
            src_down, tgt_down, ransac_result.transformation, icp_dist
        )

        fitness = icp_result.fitness
        threshold = float(self.get_parameter("icp_fitness_threshold").value)
        self.get_logger().info(
            f"ICP: fitness={fitness:.3f}, RMSE={icp_result.inlier_rmse:.5f}"
        )
        if fitness < threshold:
            raise ValueError(
                f"ICP fitness={fitness:.3f} por debajo del umbral={threshold:.3f}"
            )

        return icp_result.transformation

    @staticmethod
    def _preprocess_for_registration(cloud, voxel_size):
        """Voxel downsample → outlier removal → normal estimation."""
        down = cloud.voxel_down_sample(voxel_size)
        down, _ = down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
        )
        down.orient_normals_consistent_tangent_plane(k=10)
        return down

    @staticmethod
    def _compute_fpfh(cloud, voxel_size):
        return o3d.pipelines.registration.compute_fpfh_feature(
            cloud,
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100),
        )

    def _ransac_global_registration(self, source, target, src_fpfh, tgt_fpfh, voxel_size):
        dist_thresh = voxel_size * 1.5
        max_iter = int(self.get_parameter("ransac_max_iter").value)
        return o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source,
            target,
            src_fpfh,
            tgt_fpfh,
            mutual_filter=True,
            max_correspondence_distance=dist_thresh,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(
                False
            ),
            ransac_n=3,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist_thresh),
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(max_iter, 0.999),
        )

    def _icp_refine(self, source, target, init_transform, max_dist):
        max_iter = int(self.get_parameter("icp_max_iter").value)
        return o3d.pipelines.registration.registration_icp(
            source,
            target,
            max_dist,
            init_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter),
        )

    def _trim_model(self):
        """Voxel-downsample the model and hard-cap point count."""
        voxel = float(self.get_parameter("voxel_size").value)
        if voxel > 0.0 and len(self.model_o3d.points) > 0:
            self.model_o3d = self.model_o3d.voxel_down_sample(voxel)

        max_pts = int(self.get_parameter("max_accumulated_points").value)
        if max_pts > 0 and len(self.model_o3d.points) > max_pts:
            indices = np.random.choice(
                len(self.model_o3d.points), max_pts, replace=False
            ).tolist()
            self.model_o3d = self.model_o3d.select_by_index(indices)

    # ------------------------------------------------------------------ #
    # Cloud I/O helpers                                                    #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _to_o3d(xyz, rgb):
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
        if rgb is not None:
            cloud.colors = o3d.utility.Vector3dVector(
                rgb.astype(np.float64) / 255.0
            )
        return cloud

    @staticmethod
    def _from_o3d(cloud):
        xyz = np.asarray(cloud.points, dtype=np.float32)
        rgb = None
        if cloud.has_colors():
            rgb = (np.asarray(cloud.colors) * 255.0).clip(0, 255).astype(np.uint8)
        return xyz, rgb

    def _build_complete_cloud(self):
        if self.model_o3d is None or len(self.model_o3d.points) == 0:
            return None

        xyz, rgb = self._from_o3d(self.model_o3d)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.target_frame
        return self._create_cloud(header, xyz, rgb if self.model_has_rgb else None)

    # ------------------------------------------------------------------ #
    # TF / coordinate helpers                                              #
    # ------------------------------------------------------------------ #

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

    def _cloud_to_target_arrays(self, msg):
        field_names = [field.name for field in msg.fields]
        if not {"x", "y", "z"}.issubset(field_names):
            self.get_logger().warn("La nube recibida no tiene campos x/y/z.")
            return None

        rgb_field = None
        if "rgb" in field_names:
            rgb_field = "rgb"
        elif "rgba" in field_names:
            rgb_field = "rgba"

        read_fields = ("x", "y", "z", rgb_field) if rgb_field else ("x", "y", "z")
        points = list(point_cloud2.read_points(msg, field_names=read_fields, skip_nans=True))
        if not points:
            return None

        xyz = np.asarray([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
        rgb = None
        if rgb_field:
            rgb = np.asarray(
                [self._unpack_rgb_value(p[3]) for p in points],
                dtype=np.uint8,
            )

        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            transform = self._lookup_cloud_transform(msg.header.frame_id, msg.header.stamp)
            if transform is None:
                return None
            xyz = self._transform_xyz(xyz, transform)

        return xyz, rgb

    def _lookup_cloud_transform(self, source_frame, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=0.2),
            )
        except TransformException:
            try:
                return self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.2),
                )
            except TransformException as exc:
                self.get_logger().warn(
                    f"No pude transformar nube de '{source_frame}' a "
                    f"'{self.target_frame}': {exc}"
                )
                return None

    def _crop_object_points(self, xyz, rgb):
        if not bool(self.get_parameter("crop_to_object").value):
            return xyz, rgb
        if self.object_center is None:
            return xyz, rgb

        radius = float(self.get_parameter("object_crop_radius").value)
        if radius <= 0.0:
            return xyz, rgb

        center = np.asarray(
            [
                self.object_center.position.x,
                self.object_center.position.y,
                self.object_center.position.z,
            ],
            dtype=np.float32,
        )
        distance_sq = np.sum((xyz - center) ** 2, axis=1)
        mask = distance_sq <= radius * radius
        if rgb is None:
            return xyz[mask], None
        return xyz[mask], rgb[mask]

    @staticmethod
    def _transform_xyz(xyz, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
        rotation_matrix = np.array(
            [
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
            ],
            dtype=np.float32,
        )
        translation_vec = np.asarray(
            [translation.x, translation.y, translation.z],
            dtype=np.float32,
        )
        return (xyz @ rotation_matrix.T) + translation_vec

    @staticmethod
    def _unpack_rgb_value(value):
        if isinstance(value, (float, np.floating)):
            packed = np.float32(value).view(np.uint32)
        else:
            packed = np.uint32(value)
        return (
            (packed >> 16) & 0xFF,
            (packed >> 8) & 0xFF,
            packed & 0xFF,
        )

    @staticmethod
    def _create_cloud(header, xyz, rgb):
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        if rgb is None:
            return point_cloud2.create_cloud(header, fields, xyz.tolist())

        fields.append(
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1)
        )
        r = rgb[:, 0].astype(np.uint32)
        g = rgb[:, 1].astype(np.uint32)
        b = rgb[:, 2].astype(np.uint32)
        a = np.full(r.shape, 255, dtype=np.uint32)
        packed = ((a << 24) | (r << 16) | (g << 8) | b).view(np.float32)
        points = np.column_stack((xyz, packed))
        return point_cloud2.create_cloud(header, fields, points.tolist())

    # ------------------------------------------------------------------ #
    # Marker / trajectory helpers                                          #
    # ------------------------------------------------------------------ #

    def _tcp_orientation(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.tcp_link,
                rclpy.time.Time(),
            )
            return transform.transform.rotation
        except Exception as exc:
            self.get_logger().error(
                f"No pude leer TF del TCP '{self.tcp_link}' en '{self.target_frame}': {exc}"
            )
            return None

    def _build_marker(self, center, orientation):
        radius = float(self.get_parameter("scan_radius").value)
        angle_deg = float(self.get_parameter("scan_angle_deg").value)
        waypoint_count = max(3, int(self.get_parameter("scan_waypoints").value))
        if radius <= 0.0 or abs(angle_deg) <= 1.0e-6:
            self.get_logger().error("scan_radius y scan_angle_deg deben ser mayores que 0.")
            return None

        roll_offset = math.radians(
            float(self.get_parameter("roll_offset_deg").value)
        )
        x_axis = self._rotate(self._roll((1.0, 0.0, 0.0), roll_offset), orientation)
        y_axis = self._rotate(self._roll((0.0, 1.0, 0.0), roll_offset), orientation)
        z_axis = self._rotate(self._roll((0.0, 0.0, 1.0), roll_offset), orientation)

        start_angle = math.radians(float(self.get_parameter("scan_start_angle_deg").value))
        total_angle = math.radians(angle_deg)
        direction = self.get_parameter("scan_direction").value
        sign = -1.0 if direction == "right" else 1.0
        height_offset = float(self.get_parameter("scan_height_offset").value)

        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.ns = "scan_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(self.get_parameter("line_width").value)
        marker.color.r = 0.1
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 1.0

        for index in range(waypoint_count):
            t = index / float(waypoint_count - 1)
            angle = start_angle + sign * total_angle * t
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            marker.points.append(
                Point(
                    x=center.position.x
                    + x * x_axis[0]
                    + y * y_axis[0]
                    + height_offset * z_axis[0],
                    y=center.position.y
                    + x * x_axis[1]
                    + y * y_axis[1]
                    + height_offset * z_axis[1],
                    z=center.position.z
                    + x * x_axis[2]
                    + y * y_axis[2]
                    + height_offset * z_axis[2],
                )
            )

        return marker

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
    node = PointCloudScannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
