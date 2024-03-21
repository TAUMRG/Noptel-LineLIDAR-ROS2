#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import os
import threading
import numpy as np
import math
import struct
import traceback

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point


class Combine_node(Node):
	def __init__(self):
		super().__init__("Point_cloud_combine")
		
		# Subscription to pointcloud messages
		self.point_cloud_sub = self.create_subscription(PointCloud2, 'cloud', self.point_cloud_callback, 10)
		
		# Publisher definition for PointCloud
		self.cloud_rate     = 1/5 # seconds
		self.cloud_msg      = PointCloud2()
		self.cloud_pub      = self.create_publisher(PointCloud2, 'cloud_combined', 10)
		self.cloud_timer    = self.create_timer(self.cloud_rate, self.cloud_out)
		
		self.clouds = []
		self.point_count = 0
		self.last_msg = None

	def point_cloud_callback(self, msg):
		self.clouds += msg.data
		self.point_count += msg.width
		self.last_msg = msg

		
	# Callback for publishing pointcloud
	def cloud_out(self):
		try:
			if self.last_msg is not None:
				self.cloud_msg.header.stamp     = self.get_clock().now().to_msg()
				self.cloud_msg.header.frame_id  = "LineLidar"
				self.cloud_msg.fields           = self.last_msg.fields
				self.cloud_msg.data             = self.clouds
				self.cloud_msg.row_step         = self.last_msg.row_step
				self.cloud_msg.point_step       = self.last_msg.point_step
				self.cloud_msg.height           = 1
				self.cloud_msg.width            = self.point_count
				self.cloud_msg.is_dense         = False
				self.cloud_msg.is_bigendian     = False

				self.cloud_pub.publish(self.cloud_msg)

		except Exception as e:
			print("@cloud_out:", e)
			print(traceback.format_exc())


# Entry point for ros2
def main(args=None):
	try:
		rclpy.init()
		node = Combine_node()
		rclpy.spin(node)

	except KeyboardInterrupt:
		node.destroy_node()
		
	except Exception as e:
		print(traceback.format_exc())

	finally:
		print("Exit")
		quit()

if __name__ == "__main__":
	main()
