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

import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import ctypes


class Combine_node(Node):
	def __init__(self):
		super().__init__("Point_cloud_recorder")
		
		# Subscription to pointcloud messages
		self.point_cloud_sub = self.create_subscription(PointCloud2, 'cloud', self.point_cloud_callback, 10)
		
		self.cloud_msg      = PointCloud2()
		self.clouds = []
		self.point_count = 0
		self.last_msg = None


	# Combine received pointclouds into one
	def point_cloud_callback(self, msg):
		self.clouds += msg.data
		self.point_count += msg.width
		self.last_msg = msg

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


# Save combined pointcloud into a .ply file
def save_ply(ros_point_cloud):
	# Ref:
	# https://answers.ros.org/question/255351/how-o-save-a-pointcloud2-data-in-python/
	# https://github.com/jediofgever/PointNet_Custom_Object_Detection/blob/847303d968a23cf27e58a5a723a7a18cfca07eee/ros_inference.py#L135
	# MIT license
	# Copyright (c) 2017, Geometric Computation Group of Stanford University
	# Copyright (c) 2017 Charles R. Qi

	
	gen      = pc2.read_points(ros_point_cloud, skip_nans=True)
	int_data = list(gen)
	xyz      = np.empty((len(int_data), 3))
	rgb      = np.empty((len(int_data), 3))
	gradient = np.empty((len(int_data), 3))

	print(f"{len(int_data)} points")

	for i, x in enumerate(int_data):
		intensity = x[3]

		s = struct.pack('>f' ,intensity)
		I = struct.unpack('>l',s)[0]

		pack = ctypes.c_uint32(I).value
		r = (pack & 0x00FF0000)>> 16
		g = (pack & 0x0000FF00)>> 8
		b = (pack & 0x000000FF)

		xyz[i]      = [x[0],x[1],x[2]]
		rgb[i]      = [r/255,g/255,b/255]
		gradient[i] = [x[0]/40, abs(x[2])/40, 1-(x[0]/40)]
		
	# Make a folder for saved clouds
	if not os.path.exists("saved_clouds"):
		os.makedirs("saved_clouds")

	out_pcd  = o3d.geometry.PointCloud()
	out_pcd.points = o3d.utility.Vector3dVector(xyz)

	# Save point cloud with intensity coloring    
	out_pcd.colors = o3d.utility.Vector3dVector(rgb)
	o3d.io.write_point_cloud(f"saved_clouds/cloud_intensity_{time.time()}.ply",out_pcd)

	# Save pointcloud with distance coloring
	out_pcd.colors = o3d.utility.Vector3dVector(gradient)
	o3d.io.write_point_cloud(f"saved_clouds/cloud_gradient_{time.time()}.ply",out_pcd)

	print("Pointcloud saved")


# Entry point for ros2
def main(args=None):
	try:
		rclpy.init()
		node = Combine_node()
		rclpy.spin(node)

	except KeyboardInterrupt:
		cloud = node.cloud_msg
		node.destroy_node()
		
	except Exception as e:
		print(traceback.format_exc())

	finally:
		print("\nProcessing pointcloud")
		save_ply(cloud)
		print("Exit")
		quit()

if __name__ == "__main__":
	main()
