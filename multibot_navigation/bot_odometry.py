#!/usr/bin/env python3
# -*- coding:utf-8 -*-
#import rospy
from tkinter import W
import rclpy
import math 
#import tf
import tf2_ros
#import tf_conversions

import numpy as np
import quaternion

from math import cos, sin, sqrt, pi

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Float64

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

#class OdometryClass:
class OdometryClass(rclpy.node.Node):
  def __init__(self):

    super().__init__("bot_odometry")
    self.timer_period = 0.02
    self.group = ReentrantCallbackGroup()
    self.timer_pub = self.create_timer(self.timer_period, self.callback_timer, callback_group=self.group)
    self.odom = Odometry()
    self.pos_x = 0
    self.pos_y = 0
    self.yaw = 0  
    
    self.declare_parameter('odom_frame_id', 'odom')
    self.declare_parameter('base_frame_id', 'base_footprint')
    self.declare_parameter('print_tf', False)
    self.declare_parameter('pub_odom_topic_name', 'odom_pub')
    self.declare_parameter('sub_odom_topic_name', 'odom_sub')

    self.odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
    self.base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value
    self.print_tf   = self.get_parameter('print_tf').get_parameter_value().bool_value
  
    self.odom_pub = self.create_publisher(Odometry, self.get_parameter('pub_odom_topic_name').get_parameter_value().string_value, 1000, callback_group=self.group)
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    self.broadcaster =  tf2_ros.TransformBroadcaster(self)

    self.position_x = 0.0
    self.position_y = 0.0
    self.pose_yaw   = 0.0

    self.create_subscription(Odometry, self.get_parameter('sub_odom_topic_name').get_parameter_value().string_value, self.callback_odom, 1000)
    
  def cast_tf(self, odom):
      
      tf_stamped = TransformStamped()
      tf_stamped.header.stamp = self.get_clock().now().to_msg()
      tf_stamped.header.frame_id = self.odom_frame
      tf_stamped.child_frame_id = self.base_frame
      tf_stamped.transform.translation.x = odom.pose.pose.position.x
      tf_stamped.transform.translation.y = odom.pose.pose.position.y
      tf_stamped.transform.translation.z = 0.
      tf_stamped.transform.rotation.x = odom.pose.pose.orientation.x
      tf_stamped.transform.rotation.y = odom.pose.pose.orientation.y
      tf_stamped.transform.rotation.z = odom.pose.pose.orientation.z
      tf_stamped.transform.rotation.w = odom.pose.pose.orientation.w
      self.broadcaster.sendTransform(tf_stamped)
  
  def callback_timer(self):
    data = self.odom
    
    delta_x =  data.twist.twist.linear.x * self.timer_period
    delta_th = data.twist.twist.angular.z * self.timer_period
    
    quate = quaternion.from_euler_angles([0.0, 0.0, self.yaw+delta_th])
    
    odom                         = Odometry()
    odom.header.stamp            = self.get_clock().now().to_msg()
    odom.header.frame_id         = self.odom_frame
    odom.child_frame_id          = self.base_frame
    
    odom.pose.pose.position.x    = self.pos_x + delta_x * cos(self.yaw + (delta_th/2.))
    odom.pose.pose.position.y    = self.pos_y + delta_x * sin(self.yaw + (delta_th/2.)) 

    odom.pose.pose.orientation.x = quate.x
    odom.pose.pose.orientation.y = quate.y
    odom.pose.pose.orientation.z = quate.z
    odom.pose.pose.orientation.w = quate.w
    
    odom.twist.twist.linear.x    = data.twist.twist.linear.x 
    odom.twist.twist.angular.z   = data.twist.twist.angular.z

    odom.twist.covariance[0] = 0.01
    odom.twist.covariance[35] = 0.1

    self.odom_pub.publish(odom)
    
    self.pos_x += delta_x * cos(self.yaw + (delta_th/2.))
    self.pos_y += delta_x * sin(self.yaw + (delta_th/2.)) 
    self.yaw   += delta_th

    if self.print_tf == True:
      self.cast_tf(odom)
  
  def callback_odom(self, data):
    self.odom = data


def main(args=None):
    try:
      rclpy.init()
      node = OdometryClass()
      executor = MultiThreadedExecutor(num_threads=2)
      # rclpy.spin(node)
      executor.add_node(node)
      executor.spin()
    except KeyboardInterrupt:
      node.get_logger().info('Node Finish: bot_odometry')
      node.destroy_node()
      executor.shutdown()
    finally:
      # node.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
    main()
