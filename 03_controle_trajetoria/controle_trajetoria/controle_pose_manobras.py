import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def normalizar_angulo(angulo):
    return math.atan2(math.sin(angulo), math.cos(angulo))


def saturar(valor, limite):
    return max(-limite, min(limite, valor))


def yaw_do_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class ControlePose(Node):
    ROTACAO_INICIAL = 'ROTACAO_INICIAL'
    TRANSLACAO = 'TRANSLACAO'
    ROTACAO_FINAL = 'ROTACAO_FINAL'
    CONCLUIDO = 'CONCLUIDO'

    def __init__(self):
        super().__init__('controle_pose')

        self.declare_parameter('x', 1.0)
        self.declare_parameter('y', 1.0)
        self.declare_parameter('theta', 0.0)

        self.declare_parameter('k_linear', 0.5)         
        self.declare_parameter('k_angular', 1.5)
        self.declare_parameter('v_max', 0.3)   
        self.declare_parameter('w_max', 1.0)   

        self.declare_parameter('tol_distancia', 0.02) 
        self.declare_parameter('tol_angulo', 0.02)    

        self.declare_parameter('odom_topic', '/odom')

        p = self.get_parameter
        self.x_d = p('x').value
        self.y_d = p('y').value
        self.theta_d = p('theta').value
        self.k_v = p('k_linear').value
        self.k_w = p('k_angular').value
        self.v_max = p('v_max').value
        self.w_max = p('w_max').value
        self.tol_d = p('tol_distancia').value
        self.tol_a = p('tol_angulo').value

        self.pose = None 
        self.estado = self.ROTACAO_INICIAL

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Odometry, p('odom_topic').value, self.odom_callback, 10)
        self.create_timer(0.05, self.controle) 

        self.get_logger().info(
            f'Objetivo: x={self.x_d:.2f} m, y={self.y_d:.2f} m, '
            f'theta={math.degrees(self.theta_d):.1f}°')

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.pose = (pos.x, pos.y, yaw_do_quaternion(msg.pose.pose.orientation))

    def mudar_estado(self, novo):
        x, y, yaw = self.pose
        self.get_logger().info(
            f'{self.estado} -> {novo} | pose: x={x:.3f} y={y:.3f} '
            f'yaw={math.degrees(yaw):.1f}°')
        self.estado = novo

    def controle(self):
        if self.pose is None:
            self.get_logger().info('Aguardando odometria...', once=True)
            return

        x, y, yaw = self.pose
        dx = self.x_d - x
        dy = self.y_d - y
        distancia = math.hypot(dx, dy)
        cmd = Twist()

        if self.estado == self.ROTACAO_INICIAL:
            if distancia < self.tol_d:
                self.mudar_estado(self.ROTACAO_FINAL)
                return
            erro = normalizar_angulo(math.atan2(dy, dx) - yaw)
            if abs(erro) < self.tol_a:
                self.mudar_estado(self.TRANSLACAO)
            else:
                cmd.angular.z = saturar(self.k_w * erro, self.w_max)

        elif self.estado == self.TRANSLACAO:
            erro_d = dx * math.cos(yaw) + dy * math.sin(yaw)
            if distancia < self.tol_d:
                self.mudar_estado(self.ROTACAO_FINAL)
            else:
                cmd.linear.x = saturar(self.k_v * erro_d, self.v_max)
                if distancia > 0.1:
                    erro_a = normalizar_angulo(math.atan2(dy, dx) - yaw)
                    if erro_d < 0:
                        erro_a = normalizar_angulo(erro_a + math.pi)
                    cmd.angular.z = saturar(self.k_w * erro_a, self.w_max)

        elif self.estado == self.ROTACAO_FINAL:
            erro = normalizar_angulo(self.theta_d - yaw)
            if abs(erro) < self.tol_a:
                self.mudar_estado(self.CONCLUIDO)
            else:
                cmd.angular.z = saturar(self.k_w * erro, self.w_max)

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControlePose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
