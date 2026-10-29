"""Publica a pose exata do modelo do Gazebo em uma mensagem Odometry."""

from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class GroundtruthGazebo(Node):

    def __init__(self):
        super().__init__('groundtruth_gazebo')
        self.modelo = self.declare_parameter('modelo', 'my_robot').value
        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        self.topico_entrada = self.declare_parameter(
            'topico_entrada', '/gazebo/model_states').value
        self.topico_saida = self.declare_parameter(
            'topico_saida', '/groundtruth/odom').value
        self.pub = self.create_publisher(Odometry, self.topico_saida, 10)
        self.create_subscription(ModelStates, self.topico_entrada, self._callback, 10)
        self.alertou_ausencia = False
        self.get_logger().info(
            f'{self.topico_entrada}[{self.modelo}] -> {self.topico_saida}')

    def _callback(self, msg):
        try:
            indice = msg.name.index(self.modelo)
        except ValueError:
            if not self.alertou_ausencia:
                self.get_logger().warning(
                    f'Modelo {self.modelo!r} ainda não está em {self.topico_entrada}')
                self.alertou_ausencia = True
            return

        pose = msg.pose[indice]
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = 'chassis'
        odom.pose.pose = pose
        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    no = GroundtruthGazebo()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
