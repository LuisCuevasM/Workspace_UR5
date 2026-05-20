#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import serial


class FSRReader(Node):

    def __init__(self):
        super().__init__('fsr_reader')

        # Parámetros
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('topic', '/fsr')

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        topic = self.get_parameter('topic').value

        # Publisher
        self.pub = self.create_publisher(
            Int32MultiArray,
            '/fsr',
            10
        )


        # Serial
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.01
        )

        # Timer
        self.timer = self.create_timer(0.001, self.read_serial)

        self.get_logger().info(
            f'Leyendo FSR desde {port} @ {baudrate}'
        )

    def read_serial(self):

        try:
            if self.ser.in_waiting > 0:

                line = self.ser.readline().decode().strip()
                if line != '':

                    values = line.split(',')

                    if len(values) == 2:

                        fsr1 = int(values[0])
                        fsr2 = int(values[1])

                        msg = Int32MultiArray()

                        msg.data = [fsr1, fsr2]

                        self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'Error serial: {e}')


def main(args=None):

    rclpy.init(args=args)

    node = FSRReader()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()