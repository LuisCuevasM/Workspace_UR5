#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import torch
import cv2
import time
import os
import tempfile

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sam2.build_sam import build_sam2_video_predictor

import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="torchvision")


def depth_to_points(valid_mask, z, fx, fy, cx, cy):
    vs, us = np.where(valid_mask)
    if us.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    zz = z[vs, us]
    xx = (us.astype(np.float32) - cx) * zz / fx
    yy = (vs.astype(np.float32) - cy) * zz / fy
    return np.stack([xx, yy, zz], axis=1)


def downsample_points(pts, max_points):
    if pts.shape[0] <= max_points:
        return pts

    idx = np.random.permutation(pts.shape[0])[:max_points]
    return pts[idx]


def box_from_mask(mask, margin_px, image_shape):
    ys, xs = np.where(mask)
    if xs.size == 0:
        return None

    height, width = image_shape[:2]
    x1 = max(0, int(xs.min()) - margin_px)
    y1 = max(0, int(ys.min()) - margin_px)
    x2 = min(width - 1, int(xs.max()) + margin_px)
    y2 = min(height - 1, int(ys.max()) + margin_px)
    if x2 <= x1 or y2 <= y1:
        return None
    return np.array([x1, y1, x2, y2], dtype=np.float32)


def smooth_box(previous_box, new_box, alpha):
    if previous_box is None:
        return new_box
    return (1.0 - alpha) * previous_box + alpha * new_box


def select_click(rgb):
    import matplotlib.pyplot as plt
    # Usar un backend que permita ventanas (importante en Docker/Ubuntu)
    try:
        import matplotlib
        matplotlib.use('TkAgg') 
    except:
        pass
        
    fig, ax = plt.subplots()
    ax.imshow(rgb)
    ax.set_title("Click sobre el objeto y cierra la ventana")
    click = {"x": None, "y": None}

    def onclick(event):
        if event.xdata is not None and event.ydata is not None:
            click["x"] = int(event.xdata)
            click["y"] = int(event.ydata)
            plt.close()

    fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()
    return click["x"], click["y"]


def select_box(rgb):
    window_name = "Selecciona el objeto y presiona ENTER o SPACE"
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        x, y, w, h = cv2.selectROI(
            window_name, bgr, showCrosshair=True, fromCenter=False
        )
        cv2.destroyWindow(window_name)

        if w <= 0 or h <= 0:
            return None

        return (int(x), int(y), int(x + w), int(y + h))
    except cv2.error:
        pass

    import matplotlib
    try:
        matplotlib.use("TkAgg")
    except Exception:
        pass

    import matplotlib.pyplot as plt
    from matplotlib.widgets import RectangleSelector

    fig, ax = plt.subplots()
    ax.imshow(rgb)
    ax.set_title("Arrastra una caja sobre el objeto")
    selection = {"box": None}

    def onselect(eclick, erelease):
        if eclick.xdata is None or eclick.ydata is None:
            return
        if erelease.xdata is None or erelease.ydata is None:
            return

        x1 = int(min(eclick.xdata, erelease.xdata))
        y1 = int(min(eclick.ydata, erelease.ydata))
        x2 = int(max(eclick.xdata, erelease.xdata))
        y2 = int(max(eclick.ydata, erelease.ydata))

        if x2 <= x1 or y2 <= y1:
            return

        selection["box"] = (x1, y1, x2, y2)
        plt.close(fig)

    rect_selector = RectangleSelector(
        ax,
        onselect,
        useblit=False,
        button=[1],
        minspanx=5,
        minspany=5,
        spancoords="pixels",
        interactive=False,
    )

    # Mantener viva la referencia del selector hasta que cierre la ventana.
    fig._rectangle_selector = rect_selector
    plt.show(block=True)
    return selection["box"]

class Sam2ServiceNode(Node):
    def __init__(self):
        super().__init__("sam2_service_node")

        # Configuración de tópicos (se mantienen)
        self.rgb_topic = "/camera/camera/color/image_raw"
        self.depth_topic = "/camera/camera/aligned_depth_to_color/image_raw"
        self.cam_info_topic = "/camera/camera/aligned_depth_to_color/camera_info"
        
        # Estado del flujo
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_info = None
        self.processing_active = False # Controla si SAM2 debe ejecutarse
        self.selection_in_progress = False
        self.object_exclusion_radius_px = 12
        self.auto_update_prompt_box = True
        self.prompt_box_margin_px = 25
        self.prompt_box_smoothing_alpha = 0.65
        self.min_mask_score = 0.20
        self.video_obj_id = 1
        self.video_max_frames = 12
        self.video_rgb_frames = []
        self.video_frame_stamps = []
        self.video_frame_dir = tempfile.mkdtemp(prefix="sam2_video_frames_", dir="/tmp")
        self.video_prompt_box = None
        self.video_prompt_points = None
        self.video_prompt_labels = None

        # Carga de modelo (se mantiene)
        checkpoint = "/workspace/sam2/checkpoints/sam2.1_hiera_base_plus.pt"
        model_cfg  = "configs/sam2.1/sam2.1_hiera_b+.yaml"
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.predictor = build_sam2_video_predictor(
            model_cfg,
            checkpoint,
            device=self.device,
        )
        self.bridge = CvBridge()

        # Publicadores
        self.cloud_pub = self.create_publisher(PointCloud2, "/sam2/object_cloud", 10)
        self.scene_cloud_pub = self.create_publisher(PointCloud2, "/sam2/scene_cloud_no_object", 10)

        # SERVICIOS SOLICITADOS
        self.select_service = self.create_service(
            Trigger, "/sam2/select_object", self.handle_start_and_select
        )
        self.stop_service = self.create_service(
            Trigger, "/sam2/pause_inference", self.handle_stop
        )
        self.resume_service = self.create_service(
            Trigger, "/sam2/resume_inference", self.handle_resume
        )

        # Suscriptores Sincronizados
        self.sub_rgb = Subscriber(self, Image, self.rgb_topic)
        self.sub_depth = Subscriber(self, Image, self.depth_topic)
        self.sub_info = Subscriber(self, CameraInfo, self.cam_info_topic)
        self.sync = ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth, self.sub_info], 5, 0.05)
        self.sync.registerCallback(self.sync_callback)

        self.get_logger().info("SAM2 Service Node listo. Esperando 'Home' (/sam2/select_object)")

    def sync_callback(self, rgb_msg, depth_msg, info_msg):
        """ Solo guarda los datos. No procesa nada a menos que processing_active sea True """
        self.latest_rgb_msg = rgb_msg
        self.latest_depth_msg = depth_msg
        self.latest_info_msg = info_msg
        
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        self.latest_rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        # Si el servicio activó la inferencia, procesamos un frame
        if self.processing_active:
            self.run_inference_cycle()

    def handle_start_and_select(self, request, response):
        """ Este servicio se llama cuando el robot dice 'Estoy en Home' """
        del request

        if self.selection_in_progress:
            response.success = False
            response.message = "Ya hay una ventana de seleccion abierta."
            return response

        if self.processing_active and hasattr(self, "prompt_box"):
            self.get_logger().info(
                "Reemplazando la seleccion activa previa "
                f"{self.prompt_box.tolist()} con una nueva seleccion manual."
            )
            self.processing_active = False

        if self.latest_rgb is None:
            response.success = False
            response.message = "No hay imágenes de cámara disponibles."
            return response

        if not hasattr(self, "latest_depth_msg") or not hasattr(self, "latest_info_msg"):
            response.success = False
            response.message = "Aun no hay depth/camera_info sincronizados disponibles."
            return response

        self.selection_in_progress = True
        try:
            self.get_logger().info("Robot en Home. Abriendo selección...")
            box = select_box(self.latest_rgb)

            if box is None:
                response.success = False
                response.message = "Selección cancelada."
                return response

            self.prompt_box = np.array(box, dtype=np.float32)
            self.last_good_prompt_box = self.prompt_box.copy()
            self.reset_video_tracking(self.latest_rgb, self.latest_rgb_msg.header.stamp)
            self.processing_active = True # Activamos el procesamiento en el callback
            if not self.run_inference_cycle():
                self.processing_active = False
                response.success = False
                response.message = "No se pudo generar la nube del objeto/escena tras la seleccion."
                return response

            response.success = True
            response.message = (
                "Seleccion aplicada e inferencia publicada en "
                f"{self.prompt_box.tolist()}"
            )
            return response
        finally:
            self.selection_in_progress = False

    def handle_stop(self, request, response):
        """ Este servicio se llama para detener SAM y esperar al siguiente ciclo """
        del request
        self.processing_active = False
        self.get_logger().info("Inferencia detenida. Esperando nueva señal de Home.")
        response.success = True
        response.message = "SAM2 detenido correctamente."
        return response

    def handle_resume(self, request, response):
        """ Reanuda la inferencia usando la seleccion actual """
        del request
        if not hasattr(self, "prompt_box"):
            response.success = False
            response.message = "No hay una seleccion previa para reanudar SAM2."
            return response

        self.processing_active = True
        self.get_logger().info("Inferencia reanudada con la seleccion actual.")
        if not self.run_inference_cycle():
            self.processing_active = False
            response.success = False
            response.message = "No se pudo regenerar la nube al reanudar SAM2."
            return response
        response.success = True
        response.message = "SAM2 reanudado correctamente."
        return response

    def stamp_key(self, stamp):
        return (stamp.sec, stamp.nanosec)

    def reset_video_tracking(self, rgb, stamp):
        self.video_prompt_box = self.prompt_box.copy()
        x1, y1, x2, y2 = self.video_prompt_box
        self.video_prompt_points = np.array(
            [[0.5 * (x1 + x2), 0.5 * (y1 + y2)]],
            dtype=np.float32,
        )
        self.video_prompt_labels = np.array([1], dtype=np.int32)
        self.video_rgb_frames = [rgb.copy()]
        self.video_frame_stamps = [self.stamp_key(stamp)]

    def append_video_frame_if_new(self):
        stamp = self.stamp_key(self.latest_rgb_msg.header.stamp)
        if self.video_frame_stamps and self.video_frame_stamps[-1] == stamp:
            return

        self.video_rgb_frames.append(self.latest_rgb.copy())
        self.video_frame_stamps.append(stamp)

        if len(self.video_rgb_frames) > self.video_max_frames:
            self.video_rgb_frames = [self.video_rgb_frames[0]] + self.video_rgb_frames[
                -(self.video_max_frames - 1):
            ]
            self.video_frame_stamps = [self.video_frame_stamps[0]] + self.video_frame_stamps[
                -(self.video_max_frames - 1):
            ]

    def write_video_frames(self):
        os.makedirs(self.video_frame_dir, exist_ok=True)
        for name in os.listdir(self.video_frame_dir):
            if name.lower().endswith((".jpg", ".jpeg")):
                os.remove(os.path.join(self.video_frame_dir, name))

        for frame_idx, rgb in enumerate(self.video_rgb_frames):
            path = os.path.join(self.video_frame_dir, f"{frame_idx:05d}.jpg")
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            if not cv2.imwrite(path, bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 95]):
                raise RuntimeError(f"No se pudo escribir frame SAM2: {path}")

    def mask_from_video_predictor(self):
        if self.video_prompt_box is None:
            self.video_prompt_box = self.prompt_box.copy()
        if self.video_prompt_points is None:
            x1, y1, x2, y2 = self.video_prompt_box
            self.video_prompt_points = np.array(
                [[0.5 * (x1 + x2), 0.5 * (y1 + y2)]],
                dtype=np.float32,
            )
            self.video_prompt_labels = np.array([1], dtype=np.int32)

        self.append_video_frame_if_new()
        self.write_video_frames()

        with torch.inference_mode():
            inference_state = self.predictor.init_state(video_path=self.video_frame_dir)
            self.predictor.reset_state(inference_state)
            _, out_obj_ids, out_mask_logits = self.predictor.add_new_points_or_box(
                inference_state=inference_state,
                frame_idx=0,
                obj_id=self.video_obj_id,
                points=self.video_prompt_points,
                labels=self.video_prompt_labels,
                box=self.video_prompt_box,
            )

            target_frame_idx = len(self.video_rgb_frames) - 1
            if target_frame_idx > 0:
                out_mask_logits = None
                for out_frame_idx, out_obj_ids, out_mask_logits in self.predictor.propagate_in_video(
                    inference_state,
                    start_frame_idx=0,
                    max_frame_num_to_track=target_frame_idx + 1,
                ):
                    if out_frame_idx == target_frame_idx:
                        break

        if out_mask_logits is None:
            return None

        obj_ids = list(out_obj_ids)
        if self.video_obj_id not in obj_ids:
            return None

        obj_index = obj_ids.index(self.video_obj_id)
        return (out_mask_logits[obj_index] > 0.0).cpu().numpy().squeeze()

    def run_inference_cycle(self):
        """ Ejecuta la lógica de SAM2 y publica la nube de puntos """
        if self.latest_rgb is None or not hasattr(self, "latest_depth_msg") or not hasattr(
            self, "latest_info_msg"
        ):
            self.get_logger().warn(
                "No hay datos RGB/depth/camera_info suficientes para correr SAM2."
            )
            return False

        try:
            mask = self.mask_from_video_predictor()
        except Exception as exc:
            self.get_logger().warn(
                f"No se pudo correr SAM2 video predictor: {exc}"
            )
            return False
        if mask is None or not np.any(mask):
            self.get_logger().warn("SAM2 video predictor no devolvio una mascara valida.")
            return False

        if self.object_exclusion_radius_px > 0:
            k = 2 * self.object_exclusion_radius_px + 1
            kernel = np.ones((k, k), dtype=np.uint8)
            exclusion_mask = cv2.dilate(mask.astype(np.uint8), kernel, iterations=1) > 0
        else:
            exclusion_mask = mask
        
        # Lógica de Nube de Puntos (reutilizada de tu código)
        depth = np.asarray(self.bridge.imgmsg_to_cv2(self.latest_depth_msg, "passthrough"))
        z = depth.astype(np.float32) * 0.001 if depth.dtype == np.uint16 else depth.astype(np.float32)
        
        info = self.latest_info_msg
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        
        depth_valid = (z > 0.1) & (z < 3.0) & np.isfinite(z)
        object_valid = mask & depth_valid
        scene_valid = (~exclusion_mask) & depth_valid
        
        pts = depth_to_points(object_valid, z, fx, fy, cx, cy)
        scene_pts = depth_to_points(scene_valid, z, fx, fy, cx, cy)
        pts = downsample_points(pts, 12000)
        scene_pts = downsample_points(scene_pts, 50000)

        if pts.shape[0] < 50:
            self.get_logger().warn(f"Too few masked depth pixels: {pts.shape[0]}")
            return False

        if self.auto_update_prompt_box:
            new_box = box_from_mask(
                mask,
                self.prompt_box_margin_px,
                self.latest_rgb.shape,
            )
            if new_box is not None:
                previous_box = getattr(self, "last_good_prompt_box", self.prompt_box)
                self.prompt_box = smooth_box(
                    previous_box,
                    new_box,
                    self.prompt_box_smoothing_alpha,
                )
                self.last_good_prompt_box = self.prompt_box.copy()
                self.get_logger().debug(
                    f"Prompt box actualizado por tracking: {self.prompt_box.tolist()}"
                )

        # Publicar
        header = Header(stamp=self.latest_rgb_msg.header.stamp, frame_id=info.header.frame_id)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))
        self.scene_cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, scene_pts))
        return True
        
        # Opcional: Auto-pausar tras generar la nube si no necesitas tracking continuo
        # self.processing_active = False 

def main():
    rclpy.init()
    node = Sam2ServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
