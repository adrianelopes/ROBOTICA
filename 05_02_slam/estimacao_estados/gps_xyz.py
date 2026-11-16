"""Nó que converte o NavSatFix do GPS (lat, lon, alt) em um ponto (X, Y, Z) em metros."""

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus

from .geodesia import ConversorGps


class GpsXyz(Node):

    def __init__(self):
        super().__init__('gps_xyz')
        entrada = self.declare_parameter('topico_entrada', '/gps/fix').value
        saida = self.declare_parameter('topico_saida', '/gps/xyz').value
        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        self.origem_automatica = self.declare_parameter('origem_automatica', False).value
        self.origem = (
            self.declare_parameter('origem_lat', 0.0).value,
            self.declare_parameter('origem_lon', 0.0).value,
            self.declare_parameter('origem_alt', 0.0).value,
        )

        self.conversor = None
        if not self.origem_automatica:
            self.conversor = ConversorGps(*self.origem)

        self.pub = self.create_publisher(PointStamped, saida, 10)
        self.create_subscription(NavSatFix, entrada, self._fix_callback, qos_profile_sensor_data)
        self.get_logger().info(f'{entrada} -> {saida} (frame {self.frame_id})')

    def _fix_callback(self, msg):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if self.conversor is None:
            # Origem = primeira leitura: o (0, 0, 0) é onde o robô ligou.
            self.conversor = ConversorGps(msg.latitude, msg.longitude, msg.altitude)
            self.get_logger().info(
                f'Origem do GPS: lat={msg.latitude:.7f} lon={msg.longitude:.7f} '
                f'alt={msg.altitude:.2f}')

        x, y, z = self.conversor.para_xyz(msg.latitude, msg.longitude, msg.altitude)
        ponto = PointStamped()
        ponto.header.stamp = msg.header.stamp
        ponto.header.frame_id = self.frame_id
        ponto.point.x, ponto.point.y, ponto.point.z = x, y, z
        self.pub.publish(ponto)


def main(args=None):
    rclpy.init(args=args)
    no = GpsXyz()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
