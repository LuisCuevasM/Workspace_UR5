#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class ProbedWallSquare(Node):
    def __init__(self):
        super().__init__("probed_wall_pub")

        # --- TUS PUNTOS PROBADOS (CONFIGURACIÓN) ---
        # Copia aquí las coordenadas X e Y de tus pruebas
        self.p1 = (-0.456, 0.248) 
        self.p2 = (-0.251, 0.451)
        
        # La altura Z y la Orientación capturada
        self.center_z = 0.65
        self.orientation = {'x': 0.304, 'y': -0.628, 'z': -0.642, 'w': 0.317}

        # --- DIMENSIONES DEL CUADRADO ---
        self.side_length = 0.15   # 15 cm de lado
        self.step_size = 0.30     # 1 cm entre puntos

        # Configuración ROS
        self.topic = "/ur5/target_pose"
        self.frame_id = "base" # Ojo: en tu log dice "base", no "base_link"
        self.pub = self.create_publisher(PoseStamped, self.topic, 10)
        self.rate_hz = 5.0

        # --- CÁLCULOS AUTOMÁTICOS ---
        self.center_x, self.center_y, self.wall_yaw = self.calculate_wall_geometry()
        
        self.get_logger().info(f"Pared detectada en ángulo Yaw: {math.degrees(self.wall_yaw):.2f} grados")
        self.get_logger().info(f"Centro calculado: X={self.center_x:.3f}, Y={self.center_y:.3f}")

        # Generar trayectoria
        self.path = self.generate_path()
        self.idx = 0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_loop)

    def calculate_wall_geometry(self):
        """Calcula el centro y el ángulo basado en los dos puntos P1 y P2."""
        x1, y1 = self.p1
        x2, y2 = self.p2

        # 1. Calcular el punto medio (Centro del cuadrado)
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        # 2. Calcular el ángulo de la línea (arcotangente de delta_y / delta_x)
        # Nota: Calculamos el ángulo del vector que va de P1 a P2
        delta_x = x2 - x1
        delta_y = y2 - y1
        
        # atan2 devuelve el ángulo en radianes respecto al eje X
        yaw = math.atan2(delta_x, delta_y) 
        # NOTA MATEMÁTICA: Usamos (dx, dy) invertido aquí porque en el código de generación
        # nos movemos sobre el eje "Y local" (ancho) y queremos saber cuánto afecta a X.
        # Básicamente, estamos calculando la pendiente de la pared.
        
        # Corrección manual basada en tus datos:
        # P1->P2: X aumenta (-0.4 a -0.2), Y aumenta (0.2 a 0.4).
        # Es una diagonal a 45 grados positivos en el plano XY si miramos desde arriba.
        
        # Recalculamos el ángulo de rotación para la transformación
        # Usamos la pendiente m = dx/dy para proyectar
        return cx, cy, yaw

    def generate_path(self):
        points = []
        half = self.side_length / 2.0

        # Coordenadas locales del cuadrado (Ancho "Y", Altura "Z")
        # El centro es (0,0)
        local_corners = [
            (-half, -half), # Abajo-Izq
            ( half, -half), # Abajo-Der
            ( half,  half), # Arriba-Der
            (-half,  half), # Arriba-Izq
            (-half, -half)  # Cerrar loop
        ]

        # Vector unitario de la dirección de la pared (P1 -> P2 normalizado)
        dx = self.p2[0] - self.p1[0]
        dy = self.p2[1] - self.p1[1]
        mod = math.sqrt(dx**2 + dy**2)
        u_x = dx / mod
        u_y = dy / mod

        for i in range(len(local_corners) - 1):
            ly1, lz1 = local_corners[i]
            ly2, lz2 = local_corners[i+1]

            dist = math.sqrt((ly2 - ly1)**2 + (lz2 - lz1)**2)
            steps = int(max(dist / self.step_size, 2))

            for j in range(steps):
                alpha = j / float(steps)
                
                # Interpolación en el espacio LOCAL del cuadrado
                curr_local_width = ly1 + alpha * (ly2 - ly1) # Cuánto nos movemos a lo largo de la pared
                curr_local_z = lz1 + alpha * (lz2 - lz1)     # Cuánto subimos/bajamos

                # --- TRANSFORMACIÓN A GLOBAL ---
                # Usamos el vector unitario de la pared para proyectar el "ancho"
                # Centro Global + (Movimiento Lateral * Vector Unitario)
                final_x = self.center_x + (curr_local_width * u_x)
                final_y = self.center_y + (curr_local_width * u_y)
                final_z = self.center_z + curr_local_z

                pose = PoseStamped()
                pose.header.frame_id = self.frame_id
                pose.pose.position.x = final_x
                pose.pose.position.y = final_y
                pose.pose.position.z = final_z
                
                # Usamos TU orientación capturada
                pose.pose.orientation.x = self.orientation['x']
                pose.pose.orientation.y = self.orientation['y']
                pose.pose.orientation.z = self.orientation['z']
                pose.pose.orientation.w = self.orientation['w']

                points.append(pose)
        
        return points

    def timer_loop(self):
        if not self.path: return
        msg = self.path[self.idx]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.idx = (self.idx + 1) % len(self.path)

def main():
    rclpy.init()
    node = ProbedWallSquare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()