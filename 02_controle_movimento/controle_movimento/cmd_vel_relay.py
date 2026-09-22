"""Repassa /cmd_vel para a entrada do diff_drive_controller."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class CmdVelRelay(Node):
    """Assina /cmd_vel e publica em /diff_drive_controller/cmd_vel_unstamped."""

    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.create_subscription(Twist, '/cmd_vel', self.pub.publish, 10)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
