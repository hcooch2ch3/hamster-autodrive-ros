#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from roboid import Hamster


class HamsterDriver(Node):
    def __init__(self):
        super().__init__('hamster_driver_node')
        self.hamster = Hamster()

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription
        self.get_logger().info('Hamster driver node started.')

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        max_speed = 100
        wheel_base = 0.1

        left_speed = (linear - angular * wheel_base / 2.0) * max_speed
        right_speed = (linear + angular * wheel_base / 2.0) * max_speed

        left_speed = int(max(min(left_speed, 100), -100))
        right_speed = int(max(min(right_speed, 100), -100))

        self.get_logger().info(f'Setting wheels: left={left_speed}, right={right_speed}')
        self.hamster.wheels(left_speed, right_speed)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = HamsterDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
