"""Nó que publica a varredura do lidar 2D (LaserScan) como nuvem de pontos (PointCloud2)."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

from .nuvem import scan_para_pontos


class ScanParaNuvem(Node):

    def __init__(self):
        super().__init__('scan_para_nuvem')
        entrada = self.declare_parameter('topico_entrada', '/scan').value
        saida = self.declare_parameter('topico_saida', '/scan/points').value

        self.pub = self.create_publisher(PointCloud2, saida, 10)
        self.create_subscription(LaserScan, entrada, self._scan_callback, qos_profile_sensor_data)
        self.get_logger().info(f'{entrada} -> {saida}')

    def _scan_callback(self, msg):
        pontos = scan_para_pontos(msg.ranges, msg.angle_min, msg.angle_increment,
                                  msg.range_min, msg.range_max)
        self.pub.publish(point_cloud2.create_cloud_xyz32(msg.header, pontos))


def main(args=None):
    rclpy.init(args=args)
    no = ScanParaNuvem()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
