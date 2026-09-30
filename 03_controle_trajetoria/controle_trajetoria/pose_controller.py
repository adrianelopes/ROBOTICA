#Controle de pose contínuo

import math

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from simple_pid import PID


def normalize_angle(angle):
    """Leva um ângulo para o intervalo [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    """Extrai o yaw (rotação em z) de um quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def clamp(value, limit):
    """Limita value ao intervalo [-limit, limit]."""
    return max(-limit, min(limit, value))


class PoseController(Node):
    """Lê a pose desejada e a odometria e publica velocidades em /cmd_vel."""

    def __init__(self):
        super().__init__('pose_controller')

        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('position_tolerance', 0.03)
        self.declare_parameter('yaw_tolerance', 0.05)
        defaults = {'pid_x': (1.2, 0.0, 0.0),
                    'pid_y': (4.0, 1.0, 0.0),
                    'pid_theta': (0.2, 0.0, 0.0)}
        for name, (kp, ki, kd) in defaults.items():
            self.declare_parameter(f'{name}.kp', kp)
            self.declare_parameter(f'{name}.ki', ki)
            self.declare_parameter(f'{name}.kd', kd)

        p = self.get_parameter
        self.rate = p('control_rate').value
        self.max_lin = p('max_linear_vel').value
        self.max_ang = p('max_angular_vel').value
        self.pos_tol = p('position_tolerance').value
        self.yaw_tol = p('yaw_tolerance').value

        # Os três PIDs (9 ganhos).
        self.pid_x = self._make_pid('pid_x', self.max_lin)
        self.pid_y = self._make_pid('pid_y', self.max_ang)
        self.pid_theta = self._make_pid('pid_theta', self.max_ang)

        self.pose = None       # (x, y, yaw) atual
        self.goal = None       # (x, y, yaw) desejado
        self.reached = False   # True enquanto está dentro da tolerância
        self.last_time = None

        self.cmd_pub = self.create_publisher(
            Twist, p('cmd_vel_topic').value, 10)
        self.create_subscription(
            Odometry, p('odom_topic').value, self.odom_callback, 10)
        self.create_subscription(
            PoseStamped, p('goal_topic').value, self.goal_callback, 10)
        self.create_timer(1.0 / self.rate, self.control_loop)

        self.get_logger().info(
            f"Controle de pose pronto. Odometria: {p('odom_topic').value} | "
            f"Pose desejada: {p('goal_topic').value}")

    def _make_pid(self, prefix, limit):
        """Cria um PID com ganhos lidos dos parâmetros 'prefix.kp/ki/kd'.

        A biblioteca calcula (setpoint - entrada). Usamos setpoint 0 e
        passamos o NEGATIVO do erro, então a saída é Kp*e + Ki*int(e) + Kd*de/dt.
        sample_time=None: o dt é fornecido por nós (relógio do ROS/simulação).
        """
        p = self.get_parameter
        return PID(p(f'{prefix}.kp').value, p(f'{prefix}.ki').value,
                   p(f'{prefix}.kd').value, setpoint=0.0, sample_time=None,
                   output_limits=(-limit, limit))

    def _reset_pids(self):
        for pid in (self.pid_x, self.pid_y, self.pid_theta):
            pid.reset()

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose = (pos.x, pos.y, yaw)

    def goal_callback(self, msg):
        pos = msg.pose.position
        yaw = yaw_from_quaternion(msg.pose.orientation)
        self.goal = (pos.x, pos.y, yaw)
        self.reached = False
        self._reset_pids()
        self.get_logger().info(
            f'Nova pose desejada: x={pos.x:.2f} y={pos.y:.2f} '
            f'yaw={yaw:.2f} rad')

    def compute_command(self, pose, goal, dt):
        """Retorna (v, w) a partir da pose atual, da desejada e do dt."""
        x, y, yaw = pose
        xd, yd, yaw_d = goal

        # 1) Erro no referencial do mundo {I}
        e_x = xd - x
        e_y = yd - y
        e_theta = normalize_angle(yaw_d - yaw)

        # Chegou: para e zera os integradores (evita "wind-up" parado)
        if math.hypot(e_x, e_y) < self.pos_tol and abs(e_theta) < self.yaw_tol:
            if not self.reached:
                self.reached = True
                self._reset_pids()
                self.get_logger().info('Pose desejada alcançada.')
            return 0.0, 0.0
        self.reached = False

        # 2) Erro no referencial do robô {B}: e^b = R^-1 * e
        c, s = math.cos(yaw), math.sin(yaw)
        e_x_b = c * e_x + s * e_y
        e_y_b = -s * e_x + c * e_y

        # 3) Três PIDs simultâneos
        v = self.pid_x(-e_x_b, dt=dt)                          # frente/trás
        w = (self.pid_y(-e_y_b, dt=dt)                         # erro lateral
             + self.pid_theta(-e_theta, dt=dt))                # erro de ângulo
        return v, clamp(w, self.max_ang)

    def control_loop(self):
        if self.pose is None or self.goal is None:
            return

        now = self.get_clock().now()
        if self.last_time is None:
            dt = 1.0 / self.rate
        else:
            dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        v, w = self.compute_command(self.pose, self.goal, dt)

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        # Publica sempre (inclusive zero): o diff_drive_controller para o robô
        # se ficar cmd_vel_timeout sem receber comandos.
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PoseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
