#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

class Fakebot(Node):
    def __init__(self):
        super().__init__('fakebot')
        
        timer_period_pub = 0.02
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('print_tf', False)
        self.declare_parameter('odom_topic_name', 'odom')
        self.declare_parameter('twist_topic_name', 'cmd_vel')
        
        self.odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.print_tf   = self.get_parameter('print_tf').get_parameter_value().bool_value
        
        self.timer_pub = self.create_timer(timer_period_pub, self.callback_odom)

        self.vel_tx = 0.0
        self.vel_yz = 0.0

        self.create_subscription(Twist, self.get_parameter('twist_topic_name').get_parameter_value().string_value, self.callback_twist, 1)
        self.odom_pub = self.create_publisher(Odometry, self.get_parameter('odom_topic_name').get_parameter_value().string_value, 100)
    
    def callback_twist(self, data):
        
        self.vel_tx    = data.linear.x 
        self.vel_yz   = data.angular.z
    
    def callback_odom(self):
        odom                         = Odometry()
        odom.header.stamp            = self.get_clock().now().to_msg()
        odom.header.frame_id         = self.odom_frame
        odom.child_frame_id          = self.base_frame
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0      

        odom.twist.twist.linear.x    = self.vel_tx
        odom.twist.twist.angular.z   = self.vel_yz

        self.odom_pub.publish(odom)
    

def main(args=None):
    rclpy.init(args=args)
    node = Fakebot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()