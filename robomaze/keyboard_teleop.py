import sys
import select
import tty
import termios

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('robot_name', 'WallE')
        robot_name = self.get_parameter('robot_name').value

        self.publisher = self.create_publisher(
            Twist, f'{robot_name}/cmd_vel', 10)

        self.v = 0.0
        self.w = 0.0

        self.get_logger().info(
            f'Keyboard teleop for robot "{robot_name}"')
        self.get_logger().info(
            'Use arrow keys to control (up/down=speed, left/right=turn). '
            'Press q to quit.')

    def publish_twist(self):
        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = self.w
        self.publisher.publish(msg)

    def run(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())

            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)

                if select.select([sys.stdin], [], [], 0) == \
                        ([sys.stdin], [], []):
                    c = sys.stdin.read(1)
                    if c == '\x1b':  # ESC — arrow key sequence
                        arrow = ord(sys.stdin.read(2)[1])
                        if arrow == 65:    # up
                            self.v += 0.1
                        elif arrow == 66:  # down
                            self.v -= 0.1
                        elif arrow == 67:  # right
                            self.w -= 0.1
                        elif arrow == 68:  # left
                            self.w += 0.1
                        else:
                            self.get_logger().info('Exiting')
                            break

                        self.publish_twist()
                        self.get_logger().info(
                            f'v={self.v:.1f}, w={self.w:.1f}')

                    elif c == 'q':
                        self.get_logger().info('Exiting')
                        break
        finally:
            # Stop the robot before exiting
            self.v = 0.0
            self.w = 0.0
            self.publish_twist()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
