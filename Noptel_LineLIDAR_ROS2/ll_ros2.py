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
from sensor_msgs.msg import PointCloud2, LaserScan, PointField, PointCloud
from geometry_msgs.msg import Point, TransformStamped

#sys.path.append(".")
#sys.path.append("..")

from linelidarclass.linelidar import LineLidar, LLchr

class LineLidar_node(Node):
	def __init__(self, addr, freq):
		super().__init__("LineLidar")

		self.script = os.path.basename(__file__)
		rclpy.logging.get_logger(f'{self.script}').info("Starting")
				
		# Parameter declarations----------------------------------------------------
		
		# IP address
		self.declare_parameter('addr', '192.168.10.98' )
		self.addr = self.get_parameter('addr').get_parameter_value().string_value
		
		# Sampling rate
		self.declare_parameter('freq', 50 )
		self.freq = self.get_parameter('freq').get_parameter_value().integer_value
		
		# Threshold
		self.declare_parameter('threshold', 3000 )
		self.threshold = self.get_parameter('threshold').get_parameter_value().integer_value
	
		# Encoder resolution
		self.declare_parameter('encoder_step_count', 1200-1 )
		self.encoder_steps = self.get_parameter('encoder_step_count').get_parameter_value().integer_value
		self.encoder_scale = -(2 * math.pi)/(self.encoder_steps)


		# Start LineLidar thread
		self.t               = threading.Thread(target = self.linelidar_comm_thread, args = (self.addr, self.freq))
		self.targets         = []
		self.trigger_counter = 0
		self.t.start()

		# Publisher definition for PointCloud
		self.cloud_rate     = 1/(self.freq + 1) # seconds
		self.cloud_msg      = PointCloud2()
		self.cloud_pub      = self.create_publisher(PointCloud2, 'cloud', 10)
		self.cloud_timer    = self.create_timer(self.cloud_rate, self.cloud_out)
		
		
	# Rotation matrix to rotate the pointcloud according to quadrature encoder steps
	def R(self, phi):
		r = [[np.cos(phi), 0, np.sin(phi)],
		     [0, 1, 0],
		     [-np.sin(phi), 0, np.cos(phi)]]
		return r


	# Callback for publishing pointcloud
	def cloud_out(self):
		try:
			point_struct = struct.Struct("<ffff")
			points = []
			rotated_point = []
			intensity = []

			for target in self.targets:
				ra     = np.radians( target[1])
				dist_x = np.cos(ra) * target[0]
				dist_y = np.sin(ra) * target[0]
				points.append(Point(x=dist_x, y=dist_y, z=0.0))
				intensity.append(target[2])
	
			# Without encoder trigger_counter is 0 resulting in no rotation
			r = self.R(self.trigger_counter * self.encoder_scale)
			
			for point in points:
				rotated = (np.dot([point.x, point.y, point.z], r))
				rotated_point.append(Point(x=rotated[0], y=rotated[1], z=-rotated[2]))
				
			points = rotated_point
			size = 4
			ros_dtype = PointField.FLOAT32
			fields = [PointField(name=n, offset=i*size, datatype=ros_dtype, count=1)for i, n in enumerate(['x', 'y', 'z', 'intensity'])]

			buffer = bytearray(point_struct.size * len(points))
			for i, point in enumerate(points):
				point_struct.pack_into(buffer, i * point_struct.size, point.x, point.y, point.z, intensity[i])

			self.cloud_msg.header.stamp     = self.get_clock().now().to_msg()
			self.cloud_msg.header.frame_id  = "LineLidar"
			self.cloud_msg.fields           = fields
			self.cloud_msg.data             = buffer
			self.cloud_msg.row_step         = len(buffer)
			self.cloud_msg.point_step       = point_struct.size
			self.cloud_msg.height           = 1
			self.cloud_msg.width            = len(points)
			self.cloud_msg.is_dense         = True
			self.cloud_msg.is_bigendian     = False

			self.cloud_pub.publish(self.cloud_msg)

		except Exception as e:
			print("@cloud_out:", e)
			print(traceback.format_exc())


	# LineLidar communication based on the example "multithreaded.py"
	def linelidar_comm_thread(self, address, frequency):
		rclpy.logging.get_logger(f'{self.script}').info(f"Address: {address}")
		rclpy.logging.get_logger(f'{self.script}').info(f"Frequency: {frequency}hz")
		try:
			with LineLidar(addr = address) as ll:

				targets     = []
				last_measurementid = None

				# Disable zero results reporting, enable notification on RANGE and start active ranging
				ll.report_zero_results(False)
				ll.enable_notification(LLchr.RANGE)
				ll.set_sampling_rate(frequency)
				ll.write_chr(LLchr.NB_PEAKS, peaks = 3) 
				ll.write_chr(LLchr.AMPLITUDE_THRESHOLD, threshold = self.threshold)

				# Get calibration data
				calibrated_angles = ll.read_chr(LLchr.CALIBRATED_ANGLES).angles
				min_angle = min(calibrated_angles)
				max_angle = max(calibrated_angles)

				# Set maximums according to calibration
				ll.write_chr(LLchr.MIN_ANGLE, angle = min_angle)
				ll.write_chr(LLchr.MAX_ANGLE, angle = max_angle)
				ll.write_chr(LLchr.MAX_DISTANCE, distance = 40)
				ll.write_chr(LLchr.MIN_DISTANCE, distance = 1.5)

				# Get minimum and maximum ranging distances
				min_range = ll.read_chr(LLchr.MIN_DISTANCE).distance
				max_range = ll.read_chr(LLchr.MAX_DISTANCE).distance

				# Get minimum and maximum viewing angles
				min_view_angle = ll.read_chr(LLchr.MIN_ANGLE).angle
				max_view_angle = ll.read_chr(LLchr.MAX_ANGLE).angle

				# Calculate average resolution
				b = []
				for i in range(0,len(calibrated_angles)-1):
					b.append(abs(calibrated_angles[i] - calibrated_angles[i+1]))
				resolution = sum(b)/(len(calibrated_angles)-1)

				# Print info
				print()
				print("Settings:")
				print("  Frequency  : {:0.2f} Hz".format(frequency))
				print("  Min range  : {:0.2f} m".format(min_range))
				print("  Max range  : {:0.2f} m".format(max_range))
				print("  Min angle  : {:0.2f} deg".format(min_view_angle))
				print("  Max angle  : {:0.2f} deg".format(max_view_angle))
				print("Info:")
				print("  Resolution : {:0.2f} deg".format(resolution))
				print("  Max step   : {:0.2f} deg".format(max(b)))
				print("  Min step   : {:0.2f} deg".format(min(b)))

				# Get notifications as long as we're not told to stop
				while getattr(self.t, "do_run", True):

					# Get one RANGE notification
					notif = ll.get_notification()

					# Has the measurement ID changed?
					if last_measurementid is not None and \
						notif.measurementid != last_measurementid:
						# Clear the list of targets
						targets = []

					# Add the notification's target to this measurement's targets
					targets.extend(notif.targets)
					self.targets = targets
					self.trigger_counter = notif.triggercounter

					# Save the notification's measurement ID
					last_measurementid  = notif.measurementid
					
				# Stop sampling
				ll.stop_sampling()

		except Exception as e:
		    print("@linelidar_comm_thread:", e)


# Entry point for ros2
def main(args=None):
	try:
		rclpy.init()
		node = LineLidar_node("192.168.10.98", 20)
		rclpy.spin(node)

	except KeyboardInterrupt:
		setattr(node.t, "do_run", False)
		node.t.join()
		node.destroy_node()

	finally:
		print("Exit")
		quit()

if __name__ == "__main__":
	main()
