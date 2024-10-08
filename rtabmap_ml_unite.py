#!/usr/bin/env python3

# This python script is used to unite the map generated by the rtabmap_ros package with the identified points for battle damage generated by the ml package
# Author: C1C David Ke and C1C Elliott Kmetz, July 9th 2024
# Last Updated: July 10th 2024

# Inputs:
# 1. The odometry data from the rtabmap_ros package
# 2. The pixel data from the ml package as a 3d point.  This will be a string in the form of "x y z"
# Outputs:
# 1. A point cloud that will be published to the world frame that will overlay the centroids of the identified battle damage points on the map generated by the rtabmap_ros package

#Import the different libraries needed
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
import numpy as np

#This class will handle uniting the 
class Uniter:

	#Flag variable to run the uniting method when the sensors are directly overhead.
	overhead = False

	# Variable to store the total point cloud of the united data
	total_cloud = np.zeros((0,3))	

	# Variable to store the IDs and their corresponding label
	# id_label = np.array([0, 0])

	def __init__(self):

		#Define the subscriber to the image, camera info, and string of centroid data
		# rospy.Subscriber('image', Image, self.importimage)
		rospy.Subscriber('camera_info', CameraInfo, self.importcamerainfo)
		rospy.Subscriber('pixel_map', String, self.importpixelmap)

		#Defining subscriber to the odometry data provided by rtabmap
		rospy.Subscriber('rtabmap/odom', Odometry, self.importodom)

		#Defining publishers of point clouds for each of the directions
		self.pub = rospy.Publisher('ml_points', PointCloud2, queue_size = 10)

		self.ctrl_c = False
		rospy.on_shutdown(self.shutdownhook)
				
	def importodom(self, odom):

		#isolate the x, y, and z values of the odometry data
		x = odom.pose.pose.position.x
		y = odom.pose.pose.position.y
		z = odom.pose.pose.position.z

		self.odom = np.array([x, y, z])

	def importcamerainfo(self, camera_info):

		self.camera_info = camera_info

		#Bound the center of the image frame.
		self.x_min = camera_info.width/2 - (0.25*camera_info.width)
		self.x_max = camera_info.width/2 + (0.25*camera_info.width)
		self.y_min = camera_info.height/2 - (0.25*camera_info.height)
		self.y_max = camera_info.height/2 + (0.25*camera_info.height)

	def importpixelmap(self, pixel_map):

		#Reset the overhead flag
		self.overhead = False

		#Importing the pixel map data
		self.pixel_map = pixel_map.split(",")

		#Check the length of the pixel map data
		if len(self.pixel_map) != 4:
			rospy.logwarn("The pixel map data is not in the correct format.  It should be in the form of 'label id x y'")

		#Check if the point of interest is centered using the camera pixel width and height data
		if self.x_min < float(self.pixel_map[3]) < self.x_max and self.y_min < float(self.pixel_map[4]) < self.y_max:
			self.overhead = True

			#Take the label and id of the pixel map data
			# label = self.pixel_map[0]
			# id = self.pixel_map[1]
			# self.id_label = np.vstack((self.id_label, np.array([id, label])))

			#isolate the x, y, and z values of the pixel map data
			x = self.pixel_map[2]
			y = self.pixel_map[3]
			z = 1.5	#HARD CODED ASSUMING Z IS 1.5 METERS

			pixel_pose = np.array([x, y, z])

			#Unite the maps
			self.unite_maps(pixel_pose)

	def unite_maps(self, pixel_pose):

		if self.overhead:

			try:
				# Unite the two maps
				united_x = self.odom[0] #+ pixel_pose[0]
				united_y = self.odom[1] #+ pixel_pose[1]
				united_z = self.odom[2] + pixel_pose[2]

				# Publish the point cloud to the world frame
				point_cloud = pcl2()
				# Set the x, y, z coordinates of the point cloud
				point_cloud = np.array([united_x, united_y, united_z])
				self.total_cloud = np.vstack((self.total_cloud, point_cloud))

				header = Header()
				header.stamp = rospy.Time.now()
				header.frame_id = 'map'	#publish onto the completed map

				# create point cloud of the data from points
				united_point_cloud = pcl2.create_cloud_xyz32(header, self.total_cloud)

				#Publish the point cloud to the world frame
				self.pub.publish(united_point_cloud)

			except ValueError as e:
				rospy.logerr("Error in parsing pixel map data: %s", e)
		
		else:
			rospy.logwarn("The sensors are not overhead.  The data will not be united.")

	def shutdownhook(self):
		print("Process Shutting Down.")
		self.ctrl_c = True
			

if __name__ == '__main__':
	rospy.init_node('ml_uniter')
	c = Uniter()

	rospy.spin()