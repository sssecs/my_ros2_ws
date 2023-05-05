#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('Joy_to_velo')
        self.velo_cmd_pub = self.create_publisher(Float64MultiArray, "/velocity_controller/commands", 10)
        self.joy_input_sub = self.create_subscription(Joy, "/joy", self.publish_velo_cmd,10)
        # self.create_timer(0.1,self.publish_velo_cmd)
        self.get_logger().info("started")
    
    def publish_velo_cmd(self,cmd_recive : Joy):
        cmd_left = cmd_recive.axes[2]
        cmd_right = cmd_recive.axes[5]


        msg = Float64MultiArray()
        msg.data = [-(cmd_left-1)*5,-(cmd_right-1)*5]
        self.velo_cmd_pub.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = JoySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()