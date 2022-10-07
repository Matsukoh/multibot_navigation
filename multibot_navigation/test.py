#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration

from geometry_msgs.msg import PolygonStamped, Point32, PoseArray, Pose

from std_msgs.msg import Empty
#from lawnmower_msgs.msg import Status

import tf2_ros

import numpy as np
from scipy.spatial import distance
import math
from collections import namedtuple
import quaternion


class TestNode(Node):
    def __init__(self):
        super().__init__('test_node', namespace='mow_waypoints')
        
        time = self.get_clock().now().nanoseconds
        print(time)

if __name__ == '__main__':
    try:
        rclpy.init()
        node = TestNode()
       
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node Finish: test_node')
        node.destroy_node()
    finally:
        rclpy.shutdown()