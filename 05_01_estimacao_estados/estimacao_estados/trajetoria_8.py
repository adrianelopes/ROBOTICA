"""Nó da trajetória em 8: controla com a pose estimada pelo filtro de Kalman."""

import math

from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from simple_pid import PID

from .trajetoria import comando_realimentado, erros, referencia, yaw_from_quaternion


class Trajetoria8(Node):

    def __init__(self):
        super().__init__('trajetoria_8')
        self.A = self.declare_parameter('A', 1.0).value
        self.B = self.declare_parameter('B', 2.0).value
        self.W = self.declare_parameter('omega', 0.5).value
        self.modo = self.declare_parameter('modo', 'realimentado').value
        self.k_ff = self.declare_parameter('k_ff', 1.0).value
        self.taxa = self.declare_parameter('taxa', 50.0).value
        # pose usada no controle (EKF) e pose exata do Gazebo para medir o erro.
        topico_pose = self.declare_parameter('topico_pose', '/odometry/filtered').value
        topico_real = self.declare_parameter('topico_real', '/groundtruth/odom').value

        kp, ki, kd = self.declare_parameter('pid_x', [1.5, 0.0, 0.0]).value
        self.pid_x = PID(kp, ki, kd, setpoint=0.0, sample_time=None)
        kp, ki, kd = self.declare_parameter('pid_y', [3.0, 0.0, 0.0]).value
        self.pid_y = PID(kp, ki, kd, setpoint=0.0, sample_time=None)
        kp, ki, kd = self.declare_parameter('pid_theta', [3.0, 0.0, 0.0]).value
        self.pid_theta = PID(kp, ki, kd, setpoint=0.0, sample_time=None)

        self.pose = None        # Estimada pelo EKF: é a que o controle enxerga
        self.pose_real = None   # groundtruth do Gazebo: só para comparar
        self.t0 = None
        self.t_anterior = None
        self.volta = 0
        self.soma_est = 0.0
        self.soma_real = 0.0
        self.n = 0

        self.pub_ref = self.create_publisher(PoseStamped, '/trajetoria/desejada', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_erro_est = self.create_publisher(Vector3Stamped, '/trajetoria/erro_estimado', 10)
        self.pub_erro_real = self.create_publisher(Vector3Stamped, '/trajetoria/erro_real', 10)
        self.create_subscription(Odometry, topico_pose, self._pose_callback, 10)
        self.create_subscription(Odometry, topico_real, self._real_callback, 10)
        self.create_timer(1.0 / self.taxa, self.loop)

        self.get_logger().info(
            f'A={self.A} B={self.B} omega={self.W} modo={self.modo} | controle usa {topico_pose}')

    def _pose_callback(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))

    def _real_callback(self, msg):
        p = msg.pose.pose.position
        self.pose_real = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))

    def _publica_erro(self, pub, ex, ey, etheta):
        erro = Vector3Stamped()
        erro.header.stamp = self.get_clock().now().to_msg()
        erro.vector.x, erro.vector.y, erro.vector.z = ex, ey, etheta
        pub.publish(erro)

    def loop(self):
        agora = self.get_clock().now().nanoseconds / 1e9
        if agora == 0.0:
            return
        # Espera a estimativa e também o groundtruth do Gazebo.
        if self.pose is None or self.pose_real is None:
            self.get_logger().info('Aguardando pose dos sensores e /odom...',
                                   throttle_duration_sec=2.0)
            return
        if self.t0 is None:
            self.t0 = agora
        t = agora - self.t0

        xd, yd, theta_d, v_f, w_f = referencia(t, self.A, self.B, self.W)

        # Erro visto pelo controle e erro contra a pose exata do Gazebo.
        ref_t = (xd, yd, theta_d, v_f, w_f)
        ex, ey, etheta = erros(ref_t, self.pose)
        erro_real = erros(ref_t, self.pose_real)
        xr, yr, _ = self.pose_real
        self._publica_erro(self.pub_erro_est, ex, ey, etheta)
        self._publica_erro(self.pub_erro_real, *erro_real)

        # RMSE de posição por volta (estimado x real)
        self.soma_est += ex ** 2 + ey ** 2
        self.soma_real += (xd - xr) ** 2 + (yd - yr) ** 2
        self.n += 1
        periodo = 2 * math.pi / self.W
        if t >= (self.volta + 1) * periodo:
            self.volta += 1
            self.get_logger().info(
                f'volta {self.volta}: RMSE real = {math.sqrt(self.soma_real / self.n):.3f} m | '
                f'RMSE visto pelo controle = {math.sqrt(self.soma_est / self.n):.3f} m')
            self.soma_est = self.soma_real = 0.0
            self.n = 0

        ref = PoseStamped()
        ref.header.stamp = self.get_clock().now().to_msg()
        ref.header.frame_id = 'odom'
        ref.pose.position.x = xd
        ref.pose.position.y = yd
        ref.pose.orientation.z = math.sin(theta_d / 2)
        ref.pose.orientation.w = math.cos(theta_d / 2)
        self.pub_ref.publish(ref)

        dt = 0.0 if self.t_anterior is None else agora - self.t_anterior
        self.t_anterior = agora

        if self.modo == 'malha_aberta':
            v, w = v_f, w_f
        else:
            if dt <= 0.0:
                return
            v, w = comando_realimentado(ref_t, self.pose, self.k_ff,
                                        self.pid_x, self.pid_y, self.pid_theta, dt)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    no = Trajetoria8()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
