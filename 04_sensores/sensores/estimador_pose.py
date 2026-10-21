"""Estima a pose (x, y, yaw) do robô só com GPS e IMU, no mesmo padrão do /odom."""

import math

from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from .ruido import quaternion_para_rpy


class EstimadorPose(Node):

    def __init__(self):
        super().__init__('estimador_pose')
        saida = self.declare_parameter('topico_saida', '/pose_estimada').value
        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        # frame opcional para ver a pose estimada na árvore de transformações.
        # É um filho do odom separado de chassis: o odom -> chassis é do controlador.
        self.frame_estimado = self.declare_parameter(
            'frame_estimado', 'pose_estimada').value
        # posições em x no referencial do chassi (ver os joints do xacro)
        self.gps_x = self.declare_parameter('gps_x_chassi', -0.12).value
        self.eixo_x = self.declare_parameter('eixo_x_chassi', 0.1).value
        # Calibração: neste mundo o /gps/xyz vem com X e Y de sinal trocado em relação ao
        # Gazebo (conferido contra o /odom). Use [1.0, 1.0] se o /gps/xyz for corrigido.
        sinal = self.declare_parameter('gps_sinal_xy', [-1.0, -1.0]).value
        self.sinal_x, self.sinal_y = float(sinal[0]), float(sinal[1])

        self.yaw = None   # só há pose depois da primeira leitura da IMU

        self.pub = self.create_publisher(Odometry, saida, 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Imu, '/imu/data', self._imu_callback, qos_profile_sensor_data)
        self.create_subscription(PointStamped, '/gps/xyz', self._gps_callback, 10)
        self.get_logger().info(f'/gps/xyz + /imu/data -> {saida} | sinal do GPS: '
                               f'({self.sinal_x:+.0f}, {self.sinal_y:+.0f})')
        if self.frame_estimado:
            self.get_logger().info(
                f'Árvore: {self.frame_id} -> {self.frame_estimado} (pose estimada)')

    def _imu_callback(self, msg):
        q = msg.orientation
        _, _, self.yaw = quaternion_para_rpy(q.x, q.y, q.z, q.w)

    def _gps_callback(self, msg):
        if self.yaw is None:
            return
        # 0) acerta o sinal do GPS para o referencial do Gazebo
        gx = self.sinal_x * msg.point.x
        gy = self.sinal_y * msg.point.y
        # 1) da antena do GPS até o ponto médio do eixo das rodas (é ele que o /odom acompanha)
        d = self.eixo_x - self.gps_x
        x = gx + d * math.cos(self.yaw)
        y = gy + d * math.sin(self.yaw)
        # 2) o /odom começa em (0, 0): tira a posição inicial do eixo (robô nasce em yaw = 0)
        x -= self.eixo_x

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = 'chassis'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.pub.publish(odom)

        if self.frame_estimado:
            # mesma pose, publicada como frame próprio para a árvore de transformações
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self.frame_estimado
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    no = EstimadorPose()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()