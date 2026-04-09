import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from grasp_gen.grasp_server import GraspGenSampler, load_grasp_cfg

GRIPPER_CFG = "/workspace/GraspGen/assets/gripper_configs/robotiq.yaml"  # cambia si usas otro
PC_TOPIC = "/sam2/object_cloud"

class OneShot(Node):
    def __init__(self):
        super().__init__("graspgen_test")
        self.sub = self.create_subscription(PointCloud2, PC_TOPIC, self.cb, 10)
        self.done = False

        cfg = load_grasp_cfg(GRIPPER_CFG)
        self.sampler = GraspGenSampler(cfg)
        self.get_logger().info(f"Loaded GraspGen with gripper={cfg.data.gripper_name}")

    def cb(self, msg: PointCloud2):
        if self.done:
            return

        pts = np.array(
            [(p[0], p[1], p[2]) for p in point_cloud2.read_points(msg, field_names=("x","y","z"), skip_nans=True)],
            dtype=np.float32
        )

        self.get_logger().info(f"Got cloud: {pts.shape[0]} points")

        # downsample simple
        if pts.shape[0] > 4096:
            idx = np.random.choice(pts.shape[0], 4096, replace=False)
            pts = pts[idx]

        grasps, conf = GraspGenSampler.run_inference(
            pts,
            self.sampler,
            grasp_threshold=0.8,
            num_grasps=200,
            topk_num_grasps=50,
        )

        if len(grasps) == 0:
            self.get_logger().warn("No grasps found")
        else:
            conf = conf.cpu().numpy()
            grasps = grasps.cpu().numpy()
            self.get_logger().info(f"Found grasps: {grasps.shape[0]}, conf range [{conf.min():.3f}, {conf.max():.3f}]")
            # imprime el mejor grasp
            best = np.argmax(conf)
            self.get_logger().info(f"Best grasp conf={conf[best]:.3f}\n{grasps[best]}")

        self.done = True
        rclpy.shutdown()

def main():
    rclpy.init()
    node = OneShot()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
