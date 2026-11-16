"""
Percurso automático pela sala do SLAM, guiado pelo groundtruth (/groundtruth/odom).

O controle usa a pose exata de propósito: o robô faz exatamente o mesmo trajeto em todos
os cenários, e só o que o SLAM recebe (lidar e odometria) muda. Publica em /cmd_vel e,
ao terminar, ``True`` em /percurso/concluido (QoS transient local).
"""

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from estimacao_estados.percurso import Percurso, pontos_de_lista
from estimacao_estados.ruido import quaternion_para_rpy


class PercursoSala(Node):

    def __init__(self):
        super().__init__('percurso_sala')
        pontos = pontos_de_lista(self.declare_parameter('pontos', [0.0, 0.0]).value)
        self.percurso = Percurso(
            pontos,
            v_max=float(self.declare_parameter('v_max', 0.3).value),
            w_max=float(self.declare_parameter('w_max', 0.8).value),
            tolerancia=float(self.declare_parameter('tolerancia', 0.08).value))
        self.espera = float(self.declare_parameter('espera_inicial', 3.0).value)
        taxa = float(self.declare_parameter('taxa', 20.0).value)
        entrada = self.declare_parameter('topico_pose', '/groundtruth/odom').value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        qos_latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_fim = self.create_publisher(Bool, '/percurso/concluido', qos_latch)
        self.create_subscription(Odometry, entrada, self._pose, 10)
        self.pose = None
        self.inicio = None
        self.ultimo_indice = -1
        self.avisou_fim = False
        self.create_timer(1.0 / taxa, self._controle)
        self.pub_fim.publish(Bool(data=False))
        self.get_logger().info(
            f'{len(pontos)} pontos, começa {self.espera:.0f} s após a primeira pose')

    def _pose(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = quaternion_para_rpy(q.x, q.y, q.z, q.w)
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _controle(self):
        if self.pose is None:
            return
        agora = self.get_clock().now()
        if self.inicio is None:
            self.inicio = agora
        if (agora - self.inicio).nanoseconds * 1e-9 < self.espera:
            return

        if self.avisou_fim:
            # Percurso terminado: para de publicar, liberando o /cmd_vel para o teleop
            # (exercício 3: mover o robô depois do sequestro).
            return
        v, w = self.percurso.comando(*self.pose)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.pub.publish(cmd)

        if self.percurso.indice != self.ultimo_indice and not self.percurso.concluido:
            self.ultimo_indice = self.percurso.indice
            x, y = self.percurso.pontos[self.ultimo_indice]
            self.get_logger().info(
                f'ponto {self.ultimo_indice + 1}/{len(self.percurso.pontos)}: ({x:.1f}, {y:.1f})')
        if self.percurso.concluido and not self.avisou_fim:
            self.avisou_fim = True
            self.pub_fim.publish(Bool(data=True))
            self.get_logger().info('percurso concluído')


def main(args=None):
    rclpy.init(args=args)
    no = PercursoSala()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
