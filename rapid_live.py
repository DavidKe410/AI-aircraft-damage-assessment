#!/usr/bin/env python3

#Import the different libraries needed
import rospy
import numpy as np
# import subprocess
#TODO Import the appropriate message types that we will need
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header, Int16MultiArray, Int8
import math
import sys
import std_msgs.msg
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import tf
import json

#This class will control the sensor data post-processing to create a mapping solution
class Mapper:

	#Class variables hua
	# INPUT: Define unit vectors in sensor frame
	height_robot = 0.2	#height of the robot is 1 meter
 	
	#number of dots you want in the plane
	num_dots = rospy.get_param('~num_dots', 5)

	# plane_width = 1	#width in meters
	# plane_every = 50	#Skip this many Samples from sensors

	# Get the parameter value for plane width in meters
	plane_width = rospy.get_param('~plane_width', 0.0080)  # default value is 0.0078
	plane_every = 1	#Skip this many Samples from sensors

	#Initialize the xyz arrays to store the point cloud data
	cloud_points_r = np.array([0,0,0])
	cloud_points_u = np.array([0,0,0])
	cloud_points_l = np.array([0,0,0])

	#Counters for different sections of code
	plane_num = 0
	counter = 0

	#Plane Rejection tolerances (for VL sensors)
	dev_deg_tolerance_R = 100
	dev_deg_tolerance_L = 70
	dev_deg_tolerance_U = 70

	#Compensation for misalignments. Found experimentally.
	theta_R = 53 * np.pi / 180  # theta_L in radians.
	theta_L = 60 * np.pi / 180  # theta_L in radians
	theta_U = 50 * np.pi / 180  # theta_U in radians

	#variables to store the calculated plane equations with
	right_plane = np.zeros([1,4])
	upper_plane = np.zeros([1,4])
	left_plane = np.zeros([1,4])

	#initialize data as an empty array to store all of our incoming data hua. 
	data = np.zeros(12, dtype=Int16MultiArray)

	#Ideal yaw angle for the robot down the hallway
	ideal_yaw = 145 * np.pi / 180

	#Create two flags that describe the yaw and position data
	yaw_ready = True
	position_ready = True

	def __init__(self):
		#Defining subscriber to the tof sensors' data
		rospy.Subscriber('tof', Int16MultiArray, self.import12sensors)

		#Define the subscriber to the IMU for yaw correction
		rospy.Subscriber('imu', Imu, self.import_imu)

		#Define the subscriber to the localization network for position correction
		rospy.Subscriber('localization_data', String, self.import_localization)

		#Defining publishers of point clouds for each of the directions
		self.pub3 = rospy.Publisher('right_point_cloud', PointCloud2, queue_size = 1)
		self.pub4 = rospy.Publisher('upper_point_cloud', PointCloud2, queue_size = 1)
		self.pub5 = rospy.Publisher('left_point_cloud', PointCloud2, queue_size = 1)

		self.ctrl_c = False
		rospy.on_shutdown(self.shutdownhook)
		
	#Methods to transform the world axis in the x (C_1), y (C_2), and z (C_3)
	def C_1(self,thetaTright):
		C_1 = np.array([
		[1, 0, 0],
		[0, np.cos(thetaTright), np.sin(thetaTright)],
		[0, -np.sin(thetaTright), np.cos(thetaTright)]])
		return C_1
	
	def C_2(self,thetaTright):
		C_2 = np.array([
		[np.cos(thetaTright), 0, -np.sin(thetaTright)],
		[0, 1, 0],
		[np.sin(thetaTright), 0, np.cos(thetaTright)]])
		return C_2
	
	def C_3(self,thetaTright):
		C_3 = np.array([
		[np.cos(thetaTright), np.sin(thetaTright),0],
		[-np.sin(thetaTright), np.cos(thetaTright) , 0],
		[0, 				 0,                     1]])
		return C_3

	#Method to pull the 12 sensor data points from live data rah
	def import12sensors(self, tof):
		self.counter += 1
		#Only process the data if we have the yaw and position data
		if self.counter >= self.plane_every and self.yaw_ready and self.position_ready:
			array = np.array(tof.data)*0.001 # Convert to meters here
			self.counter = 0
			if not np.all(array == 0):
				self.data = array
				self.plot_plane()

	#Method to pull the IMU data from live data
	def import_imu(self, imu):

		#Get the yaw angle from the IMU
		quaternion = (
			imu.orientation.x,
			imu.orientation.y,
			imu.orientation.z,
			imu.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.yaw = euler[2]

		#Flag that we know the yaw and can plot planes
		self.yaw_ready = True

		#Change the right and left transformation correction angles
		self.theta_R = self.theta_R + (self.yaw - self.ideal_yaw) 
		self.theta_L = self.theta_L + (self.yaw - self.ideal_yaw)

	#Method to pull the localization data from live data
	def import_localization(self, localization):
		
		#Get the position from the localization network
		localization_data = json.loads(localization)
		position = localization_data['Tag_Position']
		self.y_position = position['y']

		#Flag that we know the position and can plot planes
		self.position_ready = True

		#Correct the right and left tof data to reflect the position off the centerline
		angle = 30*np.pi/180

		#right side correction
		self.data[0] = self.data[0] - self.y_position
		self.data[1] = self.data[1] - (self.y_position/np.cos(angle))
		self.data[2] = self.data[2] - (self.y_position/np.cos(angle))
		self.data[3] = self.data[3] - (self.y_position/np.cos(angle))

		#left side correction
		self.data[8] = self.data[8] + self.y_position
		self.data[9] = self.data[9] + (self.y_position/np.cos(angle))
		self.data[10] = self.data[10] + (self.y_position/np.cos(angle))
		self.data[11] = self.data[11] + (self.y_position/np.cos(angle))
	
	#Method to calculate pointclouds for planes created from the depth data points
	def plot_plane(self):
		###############################################################################################################################
		################################       RIGHT        ##############################################################################################
		# Defining the unit vectors
		r1_front = np.array([0, 0, 1])
		r2_ne = np.array([-np.sin(self.theta_R) ** 2, np.sin(self.theta_R) * np.cos(self.theta_R), np.cos(self.theta_R)])
		r3_nw = np.array([-np.sin(self.theta_R) ** 2, -np.sin(self.theta_R) * np.cos(self.theta_R), np.cos(self.theta_R)])
		r4_s = np.array([np.sin(self.theta_R), 0, np.cos(self.theta_R)])

		#Scale vectors with readings
		r1 = r1_front*self.data[0]
		r2 = r2_ne*self.data[2]
		r3 = r3_nw*self.data[1]
		r4 = r4_s*self.data[3]

		#Stack vectors is a matrix in preparation for executing optimization via SVD.
		r_group = np.array([r1.T,r2.T,r3.T,r4.T])

		# Fit the minimum error plane via SVD
		A = np.hstack((r_group, np.ones((r_group.shape[0], 1))))

		#Fit to array (U,S,V)
		U, S, V = np.linalg.svd(A)
		Vt = V.T
		# Least square solution is the last column of V (or the last row of V')
		v_last_col = Vt[:, -1]
		a, b, c, d = v_last_col[:4]
		#Enforce the normal vectors to go into walls not out of the walls. This is for consistency and to facilitate
		#outlier rejection via checking normal vector directions
		if c <= 0:
			a, b, c, d = -v_last_col[:4]
		# equation of the z values from the planes
		f_of_xy = lambda x, y: (-d - a * x - b * y) / c
		# Validate plane by checking its normal vector collinearity with Z sensor axis.
		# The philosophy here is that normal vectors that are not fairly aligned with the sensor XY plane
		# will be rejected.

		angle_in_radians_between_XY_normals = np.arccos((np.dot(np.array([0, 0, 1]), v_last_col[0:3])))

		angle_in_degress_between_XY_normals = (np.degrees(angle_in_radians_between_XY_normals))
		if angle_in_degress_between_XY_normals<0:
			angle_in_degress_between_XY_normals+=360

		if  ((angle_in_degress_between_XY_normals <= self.dev_deg_tolerance_R and angle_in_degress_between_XY_normals>=-self.dev_deg_tolerance_R ) or
				(angle_in_degress_between_XY_normals >= 180-self.dev_deg_tolerance_R and angle_in_degress_between_XY_normals<=180+self.dev_deg_tolerance_R)) :

			# Generate x and y coordinates, randomly
			px = self.height_robot + (-2 - self.height_robot)*np.random.rand(1, self.num_dots)
			py = -self.plane_width/2 + (self.plane_width) * np.random.rand(1, self.num_dots)
			#Get z's based on these x and y's
			pz = f_of_xy(px, py)

			#Transform the data from rows into columns
			PX = np.array([px]).reshape(self.num_dots,1)
			PY = np.array([py]).reshape(self.num_dots,1)
			PZ = np.array([pz]).reshape(self.num_dots,1)

			#Place the data into xyz array format
			self.r_points = np.hstack([PX,PY,PZ])

			#For loop to edit the values of each point on a plane
			for i in range(self.num_dots):
				#transform the axis
				self.r_points[i,:] = np.dot(np.dot(self.C_2(-np.pi/2) , self.C_1(-np.pi/2)), self.r_points[i,:])

			# "Propagate" sensors motion in Rviz (global frame)
			self.r_points[:,0] += self.plane_width*self.plane_num

			# Calibration transformation in the negative y direction
			self.r_points[:, 1] -= 0.3	# meters

			# Stack all of the planes for each direction into a single array
			self.cloud_points_r = np.vstack([self.cloud_points_r, self.r_points])
			
			# Remove first row because it is all zeros
			self.cloud_points_r = self.cloud_points_r[1:]
			#print("PRINTING RIGHT POINTS", self.cloud_points_r[:, :])

			## PUBLISH RIGHT PLANE
			# header
			header = std_msgs.msg.Header()
			header.stamp = rospy.Time.now()
			header.frame_id = 'map'
			# create pcl from points
			scaled_polygon_pcl_R = pcl2.create_cloud_xyz32(header, self.cloud_points_r)
			self.pub3.publish(scaled_polygon_pcl_R)
		else:
			print("Right plane rejected")


		###############################################################################################################################
		################################      END RIGHT        ##############################################################################################

		###############################################################################################################################
		################################       LEFT        ##############################################################################################
		# Defining the unit vectors
		r1_front = np.array([0, 0, 1])
		r2_ne = np.array([-np.sin(self.theta_L) ** 2, np.sin(self.theta_L) * np.cos(self.theta_L), np.cos(self.theta_L)])
		r3_nw = np.array([-np.sin(self.theta_L) ** 2, -np.sin(self.theta_L) * np.cos(self.theta_L), np.cos(self.theta_L)])
		r4_s = np.array([np.sin(self.theta_L), 0, np.cos(self.theta_L)])

		l1 = r1_front * self.data[8]
		l2 = r2_ne * self.data[10]
		l3 = r3_nw * self.data[9]
		l4 = r4_s * self.data[11]

		l_group = np.array([l1.T,l2.T,l3.T,l4.T])

		# Fit the minimum error plane via SVD
		A = np.hstack((l_group, np.ones((l_group.shape[0], 1))))

		# Fit to array (U,S,V)
		U, S, V = np.linalg.svd(A)
		Vt = V.T

		# Least square solution is the last column of V (or the last row of V')
		v_last_col = Vt[:, -1]
		a, b, c, d = v_last_col[:4]
		# Enforce the normal vectors to go into walls not out of the walls. This is for consistency and to facilitate
		# outlier rejection via checking normal vector directions
		if c <= 0:
			a, b, c, d = -v_last_col[:4]
		# equation of the z values from the planes
		f_of_xy = lambda x, y: (-d - a * x - b * y) / c
		# Validate plane by checking its normal vector collinearity with Z sensor axis.
		# The philosophy here is that normal vectors that are not fairly aligned with the sensor XY plane
		# will be rejected.

		angle_in_radians_between_XY_normals = np.arccos((np.dot(np.array([0, 0, -1]), v_last_col[0:3])))

		angle_in_degress_between_XY_normals = (np.degrees(angle_in_radians_between_XY_normals))
		if angle_in_degress_between_XY_normals < 0:
			angle_in_degress_between_XY_normals += 360

		if ((angle_in_degress_between_XY_normals <= self.dev_deg_tolerance_L and angle_in_degress_between_XY_normals >= -self.dev_deg_tolerance_L) or
			(angle_in_degress_between_XY_normals >= 180 - self.dev_deg_tolerance_L and angle_in_degress_between_XY_normals <= 180 + self.dev_deg_tolerance_L)):

			# Generate x and y coordinates, randomly
			# r = a + (b - a). * rand(N, 1)
			px = self.height_robot + (-2 - self.height_robot)*np.random.rand(1, self.num_dots)
			py = -self.plane_width/2 + (self.plane_width) * np.random.rand(1, self.num_dots)
			# Get z's based on these x and y's
			pz = f_of_xy(px, py)

			# Transform the data from rows into columns
			PX = np.array([px]).reshape(self.num_dots, 1)
			PY = np.array([py]).reshape(self.num_dots, 1)
			PZ = np.array([pz]).reshape(self.num_dots, 1)

			# Place the data into xyz array format
			self.l_points = np.hstack([PX, PY, PZ])

			# For loop to edit the values of each point on a plane
			for i in range(self.num_dots):
				# transform the aiis
				self.l_points[i, :] = np.dot(np.matmul(self.C_2(-np.pi/2), self.C_1(np.pi/2)), self.l_points[i, :])
				# self.l_points[i,:] = np.dot(np.dot(np.dot(self.C_3(-np.pi/2), self.l_points[i,:]), self.C_1(-np.pi/2)), self.C_2(np.pi))

			# "Propagate" sensors motion in Rviz (global frame)
			self.l_points[:, 0] += self.plane_width * self.plane_num

			# Calibration transformation in the positive y direction
			self.l_points[:, 1] += 0.55	# meters

			# Stack all of the planes for each direction into a single array (PRINTING HERE FOR DEBUGGING OF RESULTS)
			self.cloud_points_l = np.vstack([self.cloud_points_l, self.l_points])
			# Remove first row because it is all zeros
			self.cloud_points_l = self.cloud_points_l[1:]
			#print("PRINTING LEFT POINTS", self.cloud_points_l[:, :])

			## PUBLISH RIGHT PLANE
			# header
			header = std_msgs.msg.Header()
			header.stamp = rospy.Time.now()
			header.frame_id = 'map'
			scaled_polygon_pcl_L = pcl2.create_cloud_xyz32(header, self.cloud_points_l)
			self.pub5.publish(scaled_polygon_pcl_L)
		else:
			print("Left plane rejected")
		# create pcl from points

			####################################    END LEFT    ##########################################################################################
			##############################################################################################################################

		####################################    UPPER    ##########################################################################################
		##############################################################################################################################

		# Defining the unit vectors
		r1_front = np.array([0, 0, 1])
		r2_ne = np.array([-np.sin(self.theta_U) ** 2, np.sin(self.theta_U) * np.cos(self.theta_U), np.cos(self.theta_U)])
		r3_nw = np.array([-np.sin(self.theta_U) ** 2, -np.sin(self.theta_U) * np.cos(self.theta_U), np.cos(self.theta_U)])
		r4_s = np.array([np.sin(self.theta_U), 0, np.cos(self.theta_U)])

		# Scale vectors using measurements from upper group
		u1 = r1_front*self.data[4]
		u2 = r2_ne*self.data[6]
		u3 = r3_nw*self.data[5]
		u4 = r4_s*self.data[7]

		u_group = np.array([u1.T,u2.T,u3.T,u4.T])

		# Fit the minimum error plane via SVD
		A = np.hstack((u_group, np.ones((u_group.shape[0], 1))))

		# Fit to array (U,S,V)
		U, S, V = np.linalg.svd(A)
		Vt = V.T

		# Least square solution is the last column of V (or the last row of V')
		v_last_col = Vt[:, -1]
		a, b, c, d = v_last_col[:4]
		# Enforce the normal vectors to go into walls not out of the walls. This is for consistency and to facilitate
		# outlier rejection via checking normal vector directions
		if c <= 0:
			a, b, c, d = -v_last_col[:4]
		# equation of the z values from the planes
		f_of_xy = lambda x, y: (-d - a * x - b * y) / c
		# Validate plane by checking its normal vector collinearity with Z sensor axis.
		# The philosophy here is that normal vectors that are not fairly aligned with the sensor XY plane
		# will be rejected.
		angle_in_radians_between_XY_normals = np.arccos((np.dot(np.array([0, 0, -1]), v_last_col[0:3])))
		angle_in_degress_between_XY_normals = np.degrees(angle_in_radians_between_XY_normals)

		if ((angle_in_degress_between_XY_normals <= self.dev_deg_tolerance_U and angle_in_degress_between_XY_normals >= -self.dev_deg_tolerance_U) or
				(angle_in_degress_between_XY_normals >= 180 - self.dev_deg_tolerance_U and angle_in_degress_between_XY_normals <= 180 + self.dev_deg_tolerance_U)):			# Generate x and y coordinates, randomly
			px = -self.plane_width/2 + (self.plane_width) * np.random.rand(1, self.num_dots)
			py = -0.5 + 1. * np.random.rand(1, self.num_dots) # hallway width
			# self.height_robot + (1.5 - self.height_robot) * np.random.rand(1, self.num_dots)

			# Get z's based on these x and y's
			pz = f_of_xy(px, py)

			# Transform the data from rows into columns
			PX = np.array([px]).reshape(self.num_dots, 1)
			PY = np.array([py]).reshape(self.num_dots, 1)
			PZ = np.array([pz]).reshape(self.num_dots, 1)

			# Place the data into xyz array format
			self.u_points = np.hstack([PX, PY, PZ])


			# "Propagate" sensors motion in Rviz (global frame)
			self.u_points[:, 0] += self.plane_width * self.plane_num

			# Stack all of the planes for each direction into a single array (PRINTING HERE FOR DEBUGGING OF RESULTS)
			self.cloud_points_u = np.vstack([self.cloud_points_u, self.u_points])
			# Remove first row because it is all zeros
			self.cloud_points_u = self.cloud_points_u[1:]
			#print("PRINTING UPPER POINTS", self.cloud_points_u[:, :])

			## PUBLISH RIGHT PLANE
			# header
			header = std_msgs.msg.Header()
			header.stamp = rospy.Time.now()
			header.frame_id = 'map'
			scaled_polygon_pcl_U = pcl2.create_cloud_xyz32(header, self.cloud_points_u)
			self.pub4.publish(scaled_polygon_pcl_U)
		else:
			print("Upper plane rejected")

		# "Propagate" sensors motion regardless of rejecting the plane or not
		self.plane_num += 1





	#TODO Publish the PC2 Hua
	def publishPC2(self):



		print("Point Cloud Created", self.counter)

	def shutdownhook(self):
			print("Process Shutting Down.")
			self.ctrl_c = True
			
if __name__ == '__main__':
	rospy.init_node('tof_to_pc2_publisher')
	c = Mapper()

	#Define the refresh rate to match that of the VLP-16
	# rate = rospy.Rate(10) # 10hz or every 100ms

	rospy.spin()