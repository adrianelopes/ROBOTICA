"""
Odometria para o SLAM: pose exata do Gazebo com erro controlado -> TF odom -> chassis.

Substitui a TF do diff_drive_controller (o slam.launch.py desliga ``enable_odom_tf`` dele)
para que o erro da odometria seja uma variável do experimento:
``escala: 0`` = odometria groundtruth; ``escala: 1`` = valores de ``config/slam_cenarios.yaml``.
"""

import random

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from estimacao_estados.odometria import OdometriaRuidosa
from estimacao_estados.ruido import quaternion_para_rpy, rpy_para_quaternion


class OdometriaSlam(Node):

    def __init__(self):
        super().__init__('odometria_slam')
        self.frame_odom = self.declare_parameter('frame_odom', 'odom').value
        self.frame_base = self.declare_parameter('frame_base', 'chassis').value
        entrada = self.declare_parameter('topico_entrada', '/groundtruth/odom').value
        saida = self.declare_parameter('topico_saida', '/odometria_slam').value
        escala = float(self.declare_parameter('escala', 0.0).value)
        semente = int(self.declare_parameter('semente', 42).value)
        nomes = ('sigma_avanco', 'sigma_giro', 'sigma_giro_avanco', 'vies_avanco', 'vies_giro')
        base = {n: float(self.declare_parameter(n, 0.0).value) for n in nomes}

        self.odometria = OdometriaRuidosa(
            **{n: escala * v for n, v in base.items()}, rng=random.Random(semente))
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, saida, 10)
        self.create_subscription(Odometry, entrada, self._callback, 10)

        if self.odometria.sem_ruido:
            descricao = 'sem ruído (odometria = groundtruth)'
        else:
            descricao = ', '.join(f'{n}={escala * v:g}' for n, v in base.items())
        self.get_logger().info(
            f'{entrada} -> TF {self.frame_odom} -> {self.frame_base} e {saida}: {descricao}')

    def _callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = quaternion_para_rpy(q.x, q.y, q.z, q.w)
        x, y, yaw = self.odometria.atualizar(p.x, p.y, yaw)
        qx, qy, qz, qw = rpy_para_quaternion(0.0, 0.0, yaw)

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.frame_odom
        tf.child_frame_id = self.frame_base
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tf.sendTransform(tf)

        odom = Odometry()
        odom.header = tf.header
        odom.child_frame_id = self.frame_base
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = tf.transform.rotation
        odom.twist = msg.twist
        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    no = OdometriaSlam()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
