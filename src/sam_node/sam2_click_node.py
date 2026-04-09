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
    fig, ax = plt.subplots()
    ax.imshow(rgb)
    ax.set_title("Click sobre el objeto")
    click = {"x": None, "y": None}

    def onclick(event):
        if event.xdata is not None and event.ydata is not None:
            click["x"] = int(event.xdata)
            click["y"] = int(event.ydata)
            plt.close()

    fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()
    return click["x"], click["y"]


class Sam2OneInstanceNode(Node):
    def __init__(self):
        super().__init__("sam2_node")

        self.rgb_topic     = "/camera/camera/color/image_raw"
        self.depth_topic   = "/camera/camera/aligned_depth_to_color/image_raw"
        self.cam_info_topic = "/camera/camera/aligned_depth_to_color/camera_info"
        self.mask_topic    = "/sam2/mask"
        self.cloud_topic   = "/sam2/object_cloud"
        self.scene_cloud_topic = "/sam2/scene_cloud_no_object"
        self.object_exclusion_radius_px = 12

        self.initialized = False

        checkpoint = "/workspace/sam2/checkpoints/sam2.1_hiera_base_plus.pt"
        model_cfg  = "configs/sam2.1/sam2.1_hiera_b+.yaml"
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Loading SAM2 on device={self.device} ...")
        self.model     = build_sam2(model_cfg, checkpoint, device=self.device)
        self.predictor = SAM2ImagePredictor(self.model)

        self.bridge    = CvBridge()
        self.mask_pub       = self.create_publisher(Image,       self.mask_topic,       10)
        self.cloud_pub      = self.create_publisher(PointCloud2, self.cloud_topic,      10)
        self.scene_cloud_pub = self.create_publisher(PointCloud2, self.scene_cloud_topic, 10)

        self.sub_rgb   = Subscriber(self, Image,       self.rgb_topic)
        self.sub_depth = Subscriber(self, Image,       self.depth_topic)
        self.sub_info  = Subscriber(self, CameraInfo,  self.cam_info_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth, self.sub_info],
            queue_size=5,
            slop=0.05,
        )
        self.sync.registerCallback(self.callback)
        self.get_logger().info("sam2_node started.")

    def callback(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):

        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        if not self.initialized:
            self.get_logger().info("Esperando click del usuario...")
            x, y = select_click(rgb)
            if x is None:
                return
            self.click_x, self.click_y = x, y
            self.initialized = True
            self.get_logger().info(f"Click seleccionado: {x}, {y}")

        #t0 = time.perf_counter()

        with torch.inference_mode():
            self.predictor.set_image(rgb)
            #t1 = time.perf_counter()

            masks, scores, _ = self.predictor.predict(
                point_coords=np.array([[self.click_x, self.click_y]], dtype=np.float32),
                point_labels=np.array([1], dtype=np.int32),
                multimask_output=True,
            )

        if self.device == "cuda":
            torch.cuda.synchronize()
        #t2 = time.perf_counter()

        best = int(np.argmax(scores))
        mask = masks[best] > 0.5

        if self.object_exclusion_radius_px > 0:
            k = 2 * self.object_exclusion_radius_px + 1
            kernel = np.ones((k, k), dtype=np.uint8)
            exclusion_mask = cv2.dilate(mask.astype(np.uint8), kernel, iterations=1) > 0
        else:
            exclusion_mask = mask

        # Tracking: actualizar centroide
        ys, xs = np.where(mask)
        if len(xs) > 50:
            self.click_x = int(np.mean(xs))
            self.click_y = int(np.mean(ys))
        else:
            self.get_logger().warn("Objeto perdido.")
        #t3 = time.perf_counter()

        # Publish mask
        mask_out = self.bridge.cv2_to_imgmsg(mask.astype(np.uint8) * 255, encoding="mono8")
        mask_out.header = rgb_msg.header
        self.mask_pub.publish(mask_out)
        #t4 = time.perf_counter()

        # Depth → Point Cloud
        depth = np.asarray(self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough"))
        z = depth.astype(np.float32) * 0.001 if depth.dtype == np.uint16 else depth.astype(np.float32)

        fx, fy = info_msg.k[0], info_msg.k[4]
        cx, cy = info_msg.k[2], info_msg.k[5]

        depth_valid = (z > 0.1) & (z < 3.0) & np.isfinite(z)
        object_valid = mask & depth_valid
        scene_valid = (~exclusion_mask) & depth_valid

        pts = depth_to_points(object_valid, z, fx, fy, cx, cy)
        scene_pts = depth_to_points(scene_valid, z, fx, fy, cx, cy)

        if pts.shape[0] < 50:
            self.get_logger().warn(f"Too few masked depth pixels: {pts.shape[0]}")
            return

        #t5 = time.perf_counter()

        # Downsample para RViz / consumidores
        pts = downsample_points(pts, max_points=12000)
        scene_pts = downsample_points(scene_pts, max_points=50000)

        #print(
        #    f"[TIMING] "
        #    f"set_img={(t1-t0)*1000:.1f}ms | "
        #    f"SAM2={(t2-t1)*1000:.1f}ms | "
        #    f"tracking={(t3-t2)*1000:.1f}ms | "
        #    f"mask_pub={(t4-t3)*1000:.1f}ms | "
        #    f"pointcloud={(t5-t4)*1000:.1f}ms | "
        #    f"TOTAL={(t5-t0)*1000:.1f}ms"
        #)

        # Publish PointCloud2
        header = Header()
        header.stamp    = rgb_msg.header.stamp
        header.frame_id = info_msg.header.frame_id

        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))
        self.scene_cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, scene_pts))
        #self.get_logger().info(f"Published cloud: {pts.shape[0]} pts | score={scores[best]:.3f}")


def main():
    rclpy.init()
    node = Sam2OneInstanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
