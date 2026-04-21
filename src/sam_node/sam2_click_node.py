#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import torch
import cv2
import time

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

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

        # Carga de modelo (se mantiene)
        checkpoint = "/workspace/sam2/checkpoints/sam2.1_hiera_base_plus.pt"
        model_cfg  = "configs/sam2.1/sam2.1_hiera_b+.yaml"
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = build_sam2(model_cfg, checkpoint, device=self.device)
        self.predictor = SAM2ImagePredictor(self.model)
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

        if self.processing_active and hasattr(self, "click_x") and hasattr(self, "click_y"):
            response.success = True
            response.message = (
                f"Ya existe una seleccion activa en ({self.click_x}, {self.click_y})."
            )
            return response

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
            x, y = select_click(self.latest_rgb)

            if x is None:
                response.success = False
                response.message = "Selección cancelada."
                return response

            self.click_x, self.click_y = x, y
            self.processing_active = True # Activamos el procesamiento en el callback
            if not self.run_inference_cycle():
                self.processing_active = False
                response.success = False
                response.message = "No se pudo generar la nube del objeto/escena tras la seleccion."
                return response

            response.success = True
            response.message = f"Seleccion aplicada e inferencia publicada en ({x}, {y})"
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
        if not hasattr(self, "click_x") or not hasattr(self, "click_y"):
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

    def run_inference_cycle(self):
        """ Ejecuta la lógica de SAM2 y publica la nube de puntos """
        if self.latest_rgb is None or not hasattr(self, "latest_depth_msg") or not hasattr(
            self, "latest_info_msg"
        ):
            self.get_logger().warn(
                "No hay datos RGB/depth/camera_info suficientes para correr SAM2."
            )
            return False

        with torch.inference_mode():
            self.predictor.set_image(self.latest_rgb)
            masks, scores, _ = self.predictor.predict(
                point_coords=np.array([[self.click_x, self.click_y]], dtype=np.float32),
                point_labels=np.array([1], dtype=np.int32),
                multimask_output=True,
            )

        best = int(np.argmax(scores))
        mask = masks[best] > 0.5

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
