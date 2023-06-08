#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import numpy as np
from control_msgs.msg import DynamicJointState

class HomeMadeOdom():

    x = 0
    y = 0
    theta = 0
    __WheelDiameter = 0.1
    __RobotWidth = 0.39
    __AngleHistoryL = 0
    __AngleHistoryR = 0

    def __init__(self):

        pass
    def update_odom(self,AngleL,AngleR):
        DistanceL = self.__WheelDiameter/2 * (AngleL - self.__AngleHistoryL)
        DistanceR = self.__WheelDiameter/2 * (AngleR - self.__AngleHistoryR)
        DistanceM = (DistanceR+DistanceL) / 2
        self.__AngleHistoryL = AngleL
        self.__AngleHistoryR = AngleR

        self.x = self.x + DistanceM * math.cos(self.theta)
        self.y = self.y + DistanceM * math.sin(self.theta)

        self.theta = self.theta + (DistanceR - DistanceL) / self.__RobotWidth

        self.theta = np.sign(self.theta) * (abs(self.theta)%(2*math.pi))






    


class DiffDriveOdomNode(Node):
    
    p3d_theta = 0
    odom_theta = 0

    p3d_x = 0
    p3d_y = 0

    odom_x = 0
    odom_y = 0
    
    
    def __init__(self):
        

        self.toybox1_home_made_odom = HomeMadeOdom()
        super().__init__('toybox1_odom')

        self.p3d_odom_sub = self.create_subscription(Odometry, "/toybox1/p3d_demo", self.PrintP3DPos ,10)
        self.wheel_odom_sub = self.create_subscription(DynamicJointState, "/dynamic_joint_states", self.PrintWheelOdom ,10)
        self.get_logger().info("started")

    def euler_from_quaternion(self,x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


    def PrintP3DPos(self,msg : Odometry):
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y

        quat_x = msg.pose.pose.orientation.x
        quat_y = msg.pose.pose.orientation.y
        quat_z = msg.pose.pose.orientation.z
        quat_w = msg.pose.pose.orientation.w
        self.p3d_theta = math.degrees( self.euler_from_quaternion(quat_x, quat_y, quat_z, quat_w)[2] )
        # self.get_logger().info('real angle: %f feak angle: %f' % (self.p3d_theta,self.odom_theta,))

        self.get_logger().info('real: %f,%f,%f feak: %f,%f,%f' % (pos_x,pos_y,self.p3d_theta, self.odom_x,self.odom_y,self.odom_theta))
 
        
    def PrintWheelOdom(self, msg : DynamicJointState):
        pos_LWheel = msg.interface_values[0].values[0]
        pos_RWheel = msg.interface_values[1].values[0]
        self.toybox1_home_made_odom.update_odom( pos_LWheel,pos_RWheel)
        self.odom_theta = math.degrees( self.toybox1_home_made_odom.theta )
        self.odom_x = self.toybox1_home_made_odom.x
        self.odom_y = self.toybox1_home_made_odom.y
        # self.get_logger().info('L: %f R: %f' % (pos_LWheel,pos_RWheel,))




    
    



def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveOdomNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()