"""Nó que acrescenta ruído gaussiano à orientação da IMU."""

import copy
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .ruido import perturbar_quaternion


class ImuOrientacao(Node):

    def __init__(self):
        super().__init__('imu_orientacao')
        entrada = self.declare_parameter('topico_entrada', '/imu/data_bruto').value
        saida = self.declare_parameter('topico_saida', '/imu/data').value
        desvio = self.declare_parameter(
            'desvio_orientacao', [0.005, 0.005, 0.01]).value     # roll, pitch, yaw (rad)
        escala = float(self.declare_parameter('escala_ruido', 1.0).value)
        semente = int(self.declare_parameter('semente', -1).value)

        self.desvio = [float(d) * escala for d in desvio]
        self.desvio_gyro = float(self.declare_parameter('desvio_gyro', 0.01).value) * escala
        self.rng = random.Random(None if semente < 0 else semente)

        self.pub = self.create_publisher(Imu, saida, 10)
        self.create_subscription(Imu, entrada, self._imu_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'{entrada} -> {saida} | desvio da orientação (rpy, rad): {self.desvio}')

    def _imu_callback(self, msg):
        q = msg.orientation
        saida = copy.deepcopy(msg)
        (saida.orientation.x, saida.orientation.y,
         saida.orientation.z, saida.orientation.w) = perturbar_quaternion(
            (q.x, q.y, q.z, q.w), self.desvio, self.rng)
                # piso de 1 mrad: com escala_ruido = 0 a covariância seria zero e o filtro de Kalman
        # confiaria cegamente na medida
        d_roll, d_pitch, d_yaw = (max(d, 1e-3) for d in self.desvio)
        saida.orientation_covariance = [
            d_roll ** 2, 0.0, 0.0,
            0.0, d_pitch ** 2, 0.0,
            0.0, 0.0, d_yaw ** 2,
        ]
        d_gyro = max(self.desvio_gyro, 1e-3)
        saida.angular_velocity_covariance = [
            d_gyro ** 2, 0.0, 0.0,
            0.0, d_gyro ** 2, 0.0,
            0.0, 0.0, d_gyro ** 2,
        ]
        self.pub.publish(saida)


def main(args=None):
    rclpy.init(args=args)
    no = ImuOrientacao()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
