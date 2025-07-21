#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import sys
import termios
import tty


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Teleop Node Started. Use WASD keys to move. (Q to quit)")

        # 속도 설정
        self.linear_speed = 0.2
        self.angular_speed = 0.5

        self.keyboard_loop()

    def keyboard_loop(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        try:
            while True:
                key = sys.stdin.read(1)
                if key == 'q':
                    break

                twist = Twist()

                if key == 'w':
                    twist.linear.x = self.linear_speed
                elif key == 's':
                    twist.linear.x = -self.linear_speed
                elif key == 'a':
                    twist.angular.z = self.angular_speed
                elif key == 'd':
                    twist.angular.z = -self.angular_speed

                self.publisher_.publish(twist)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self.get_logger().info("Teleop Node stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
