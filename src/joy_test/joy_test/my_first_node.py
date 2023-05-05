#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node

# import sensor_msgs.msg._joy

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('JoyToVelo')
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("hi")

def main(args=None):
    rclpy.init(args=args)
    node = JoySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()