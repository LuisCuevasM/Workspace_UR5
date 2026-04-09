import os, json, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

OUT_DIR = "/workspace/realrobot_pc/final"   # igual que el demo
TOPIC   = "/sam2/object_cloud"

class Saver(Node):
    def __init__(self):
        super().__init__("pc_saver")
        os.makedirs(OUT_DIR, exist_ok=True)
        self.sub = self.create_subscription(PointCloud2, TOPIC, self.cb, 10)
        self.saved = False
        self.get_logger().info(f"Waiting for {TOPIC} ...")

    def cb(self, msg: PointCloud2):
        if self.saved:
            return

        pts = []
        cols = []
        fields = [f.name for f in msg.fields]
        has_rgb = ("rgb" in fields) or ("rgba" in fields)

        for p in point_cloud2.read_points(msg, skip_nans=True):
            x,y,z = float(p[0]), float(p[1]), float(p[2])
            pts.append([x,y,z])

            if has_rgb:
                # rgb packed float or uint32 depending on publisher; handle common float32 packed
                rgb = p[3]
                if isinstance(rgb, float):
                    import struct
                    i = struct.unpack('I', struct.pack('f', rgb))[0]
                else:
                    i = int(rgb)
                r = (i >> 16) & 255
                g = (i >> 8) & 255
                b = i & 255
                cols.append([r,g,b])
            else:
                cols.append([200,200,200])  # gris por defecto

        data = {
            "pc": pts,
            "pc_color": cols,
            "grasp_poses": [],
            "grasp_conf": []
        }

        fname = os.path.join(OUT_DIR, f"obj_{int(time.time())}.json")
        with open(fname, "w") as f:
            json.dump(data, f)

        self.get_logger().info(f"Saved {len(pts)} points to {fname}")
        self.saved = True
        rclpy.shutdown()

def main():
    rclpy.init()
    rclpy.spin(Saver())

if __name__ == "__main__":
    main()
