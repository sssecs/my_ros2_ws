#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
import math

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('Joy_to_velo')
        self.velo_cmd_pub = self.create_publisher(Float64MultiArray, "/velocity_controller/commands", 10)
        self.joy_input_sub = self.create_subscription(Joy, "/joy", self.PublishVeloCmd,10)
        # self.create_timer(0.1,self.publish_velo_cmd)
        self.get_logger().info("started")

    def CloseTo(self,x1,x2):
        if abs(x1-x2)<0.001:
            return True
        else:
            return False
    
    def PublishVeloCmd(self,cmd_recive : Joy):
        cmd_left_joy = cmd_recive.axes[0]
        cmd_right_trigger = -(cmd_recive.axes[5]-1)
        cmd_left_trigger = -(cmd_recive.axes[2]-1)

        if self.CloseTo(cmd_left_trigger,0):
            velo = cmd_right_trigger * 0.25
        else:
            velo = - cmd_left_trigger  * 0.25

        omega = cmd_left_joy * math.pi

        L = 0.39
        R = 0.05

        velo_right_wheel = (2*velo + omega*L) / (2*R)
        velo_left_wheel = (2*velo - omega*L) / (2*R)


        msg = Float64MultiArray()
        msg.data = [velo_left_wheel,velo_right_wheel]
        self.velo_cmd_pub.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = JoySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()