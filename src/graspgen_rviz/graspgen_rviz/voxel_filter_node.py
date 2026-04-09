#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import time

# PointCloud2 helpers (ROS2)
from sensor_msgs_py import point_cloud2 as pc2

def pointcloud2_to_xyz_rgb(msg: PointCloud2):
    """Extrae XYZ y RGB (si existe) de un PointCloud2."""
    field_names = [f.name for f in msg.fields]
    has_rgb = ("rgb" in field_names) or ("rgba" in field_names)

    xyz = []
    rgb = [] if has_rgb else None

    if has_rgb:
        rgb_field = "rgb" if "rgb" in field_names else "rgba"
        for p in pc2.read_points(msg, field_names=("x", "y", "z", rgb_field), skip_nans=True):
            xyz.append((p[0], p[1], p[2]))
            
            # --- CORRECCIÓN AQUÍ ---
            c = p[3]
            # Detecta tanto float normal como float de numpy
            if isinstance(c, (float, np.floating)):
                # .view(np.uint32) es más eficiente y seguro que tobytes()
                c = np.float32(c).view(np.uint32)
            else:
                c = int(c)
            
            r_ = (c >> 16) & 0xFF
            g_ = (c >> 8) & 0xFF
            b_ = c & 0xFF
            rgb.append((r_, g_, b_))
    else:
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            xyz.append((p[0], p[1], p[2]))

    xyz = np.asarray(xyz, dtype=np.float32)
    if rgb is None:
        return xyz, None
    rgb = np.asarray(rgb, dtype=np.uint8)
    return xyz, rgb

def create_pointcloud2(header: Header, xyz: np.ndarray, rgb: np.ndarray = None) -> PointCloud2:
    """Empaqueta arrays de NumPy (XYZ y RGB) de vuelta a un mensaje PointCloud2."""
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    if rgb is not None:
        # RViz espera que el campo sea tipo FLOAT32 (capricho heredado de PCL)
        fields.append(PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1))
        
        # Extraer canales
        r = np.asarray(rgb[:, 0], dtype=np.uint32)
        g = np.asarray(rgb[:, 1], dtype=np.uint32)
        b = np.asarray(rgb[:, 2], dtype=np.uint32)
        
        # Agregar canal Alpha (255 = totalmente opaco)
        a = np.full(r.shape, 255, dtype=np.uint32)
        
        # Empaquetar ARGB en un entero de 32 bits
        rgb_packed = (a << 24) | (r << 16) | (g << 8) | b
        
        # "Disfrazar" el array de enteros como float32 para que RViz lo acepte
        rgb_float = rgb_packed.view(np.float32)
        
        # Unir XYZ y RGB
        points = np.column_stack((xyz, rgb_float))
    else:
        points = xyz

    return pc2.create_cloud(header, fields, points.tolist())

class VoxelFilterNode(Node):
    def __init__(self):
        super().__init__('voxel_filter_node')

        # --- Parámetros ---
        self.declare_parameter("input_topic", "/camera/camera/depth/color/points")
        self.declare_parameter("output_topic", "/camera/camera/depth/color/points_filtered")
        self.declare_parameter("voxel_size", 0.005) # 1 cm por defecto

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        # --- Suscriptores y Publicadores ---
        self.sub = self.create_subscription(PointCloud2, input_topic, self.pc_callback, 10)
        self.pub = self.create_publisher(PointCloud2, output_topic, 10)

        self.get_logger().info(f"Voxel Filter Node iniciado.")
        self.get_logger().info(f"Escuchando en: {input_topic}")
        self.get_logger().info(f"Publicando en: {output_topic}")

    def pc_callback(self, msg: PointCloud2):
        voxel_size = float(self.get_parameter("voxel_size").value)
        
        # 1. Extraer puntos
        xyz, rgb = pointcloud2_to_xyz_rgb(msg)
        if xyz.size == 0:
            return

        original_count = xyz.shape[0]

        # 2. Aplicar Voxel Downsample con NumPy
        if voxel_size > 0:
            # Dividimos por el tamaño del voxel y redondeamos hacia abajo para crear una "cuadrícula"
            voxel_coords = np.floor(xyz / voxel_size).astype(np.int32)
            
            # np.unique nos devuelve los índices de la primera vez que aparece cada voxel
            _, unique_indices = np.unique(voxel_coords, axis=0, return_index=True)
            
            xyz_filtered = xyz[unique_indices]
            rgb_filtered = rgb[unique_indices] if rgb is not None else None
        else:
            xyz_filtered = xyz
            rgb_filtered = rgb

        filtered_count = xyz_filtered.shape[0]

        # 3. Publicar la nueva nube
        out_msg = create_pointcloud2(msg.header, xyz_filtered, rgb_filtered)
        self.pub.publish(out_msg)

        # Log para ver cuánto se redujo (puedes comentarlo si hace mucho spam)
        # self.get_logger().info(f"Puntos reducidos de {original_count} a {filtered_count} (Voxel: {voxel_size}m)")

def main(args=None):
    rclpy.init(args=args)
    node = VoxelFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()