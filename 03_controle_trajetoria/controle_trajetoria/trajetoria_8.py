import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist

import math
from controle_trajetoria.trajetoria import referencia, yaw_from_quaternion, normaliza_angulo

from simple_pid import PID

class Trajetoria8(Node):
    def __init__(self):
        super().__init__('trajetoria_8')
        self.pose = None
        self.t0 = None
        self.soma_e2 = 0.0     
        self.n = 0            
        self.volta = 0
        self.get_logger().info('trajetoria_8 start')
        self.declare_parameter('A', 1.0)
        self.A = self.get_parameter('A').value
        self.declare_parameter('B', 2.0)
        self.B = self.get_parameter('B').value
        self.declare_parameter('omega', 0.5)
        self.W = self.get_parameter('omega').value
        self.declare_parameter('modo', 'malha_aberta')
        self.modo = self.get_parameter('modo').value
        self.declare_parameter('k_ff', 1.0)
        self.k_ff = self.get_parameter('k_ff').value

        self.declare_parameter('pid_x', [1.0, 0.0, 0.0])
        self.declare_parameter('pid_y', [1.0, 0.0, 0.0])
        self.declare_parameter('pid_theta', [1.0, 0.0, 0.0])

        kp, ki, kd = self.get_parameter('pid_x').value
        self.pid_x = PID(kp, ki, kd, setpoint=0.0, sample_time=None)
        kp, ki, kd = self.get_parameter('pid_y').value
        self.pid_y = PID(kp, ki, kd, setpoint=0.0, sample_time=None)
        kp, ki, kd = self.get_parameter('pid_theta').value
        self.pid_theta = PID(kp, ki, kd, setpoint=0.0, sample_time=None)

        self.t_anterior = None
        
        self.declare_parameter('taxa', 50.0)
        self.taxa = self.get_parameter('taxa').value

        self.pub_ref = self.create_publisher(PoseStamped, '/trajetoria/desejada', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(1.0/self.taxa, self.loop)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info(f'A={self.A} B={self.B} omega={self.W} modo={self.modo}')

    def loop(self):
        agora = self.get_clock().now().nanoseconds / 1e9
        if agora == 0.0:
            return
        if self.pose is None:
            self.get_logger().info('Aguardando odometria...')
            return
        if self.t0 is None:
            self.t0 = agora
        t = agora - self.t0


        xd, yd, theta_d, v_f, w_f = referencia(t, self.A, self.B, self.W)

        x, y, theta = self.pose
        ex = xd - x
        ey = yd - y
        etheta = normaliza_angulo(theta_d - theta)

        self.soma_e2 += ex**2 + ey**2
        self.n += 1
        periodo = 2 * math.pi / self.W
        if t >= (self.volta + 1) * periodo:
            rmse = math.sqrt(self.soma_e2 / self.n)
            self.get_logger().info(f'volta {self.volta + 1}: RMSE posição = {rmse:.3f} m')
            self.volta += 1
            self.soma_e2 = 0.0
            self.n = 0

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = xd
        msg.pose.position.y = yd
        msg.pose.orientation.z = math.sin(theta_d/2)
        msg.pose.orientation.w = math.cos(theta_d/2)
        self.pub_ref.publish(msg)

        dt = 0.0 if self.t_anterior is None else agora - self.t_anterior
        self.t_anterior = agora

        if self.modo == 'malha_aberta':
            v, w, = v_f, w_f
        else:
            if dt <= 0.0:
                return

            exb = math.cos(theta) * ex + math.sin(theta) * ey
            eyb = -math.sin(theta) * ex + math.cos(theta) * ey

            v = self.k_ff * v_f + self.pid_x(-exb, dt=dt)
            w = self.k_ff * w_f + self.pid_y(-eyb, dt=dt) + self.pid_theta(-etheta, dt=dt)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)
    
    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))
        # self.get_logger().info(f'odom: x={self.pose[0]:.2f} y={self.pose[1]:.2f} theta={self.pose[2]:.2f}', throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = Trajetoria8()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
