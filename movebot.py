#!/usr/bin/env python
import rospy, time
import argparse
import numpy as np
import os
import tf
import thread

# ROS Image message
from sensor_msgs.msg import Image
# ROS Laser message
from sensor_msgs.msg import LaserScan
# ROS Geometry/Velocity message
from geometry_msgs.msg import Twist, Pose, Quaternion
# ROS String message
from std_msgs.msg import String
# ROS Odometry(Pose/Position) message
from nav_msgs.msg import Odometry
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
# Instantiate CvBridge
bridge = CvBridge()

def vel_msg_init():
	vel_msg = Twist()
	vel_msg.linear.x=0
	vel_msg.linear.y=0
	vel_msg.linear.z=0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
	return vel_msg

## this function is called via thread to keep the processing out of the subscriber/callback functions
def threaded_image_handling(cv2_img):
	global turn, recognition_running
	print("Processing Image data")
	cv2.imwrite('camera_img.jpeg', cv2_img)	## save image
	##call your recognition model here and set the value of turn
	turn = "na"
	recognition_running = False

## this subscriber function is called every ~millisecond with the bot view image as msg
def image_callback(msg):
	print("Received an image!")
	global recognition_running
	if recognition_running == True:
		print("Waiting for previous recognition thread to finish processing")
	else:
		global turn		##accessing global variable turn
		if turn=="na":
			recognition_running = True
			try:
				# Convert your ROS Image message to OpenCV2
				cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
			except CvBridgeError, e:
				print(e)
				recognition_running = False
			else:
				print("Starting an image handling thread")
				thread.start_new_thread(threaded_image_handling, (cv2_img,))
				# Do image processing inside the thread
		else:
			## havent completed a detected turn yet, therefore no need to run the recognition pipeline
			print("---------Waiting for", turn,"turn to be completed--------")


## this subscriber function is called every ~millisecond with the bot's orientation and pose
# def odom_callback(msg):
# 	# print("position:",msg.pose.pose.position)
# 	# print("orientation:",msg.pose.pose.orientation)
# 	(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
# 	## yaw relects orientation of the bot
# 	## 1.5708 = 90 degrees to left
# 	## -1.5708 = 90 degrees to right
# 	## 0 = 0 degrees
# 	## +-3.14159 (PI) = 180 degrees
# 	## for level 1 and 3 the bot follows staright lines, therefore we make its orientation either one of these
# 	## This is because the right turn may not be perfect and even a new orientation of 88 degrees will change the bot's course
# 	pos = np.array((0, 1.5708, -1.5708, 3.14159, -3.14159))
# 	global laser_running
# 	if laser_running == False:
# 		o = pos[np.argmin(abs(pos-yaw))]		## get which orientation is the closest

# 		#### Turn bot to correct orientation ####
# 		##
# 		##

# 		#### Make angular velocity 0 again ####
# 		vel_msg = vel_msg_init()
# 		vel_msg.linear.x = linear_vel
# 		velocity_publisher.publish(vel_msg)


## this function is called via thread to keep the processing out of the subscriber/callback functions
def threaded_obstacle_handling(d):
	## This code only works for wall right ahead of the bot (since we are only seeing d[0])
	## you should get appropriate distance and threshold according to wall placement for turn
	dist = d[0]
	thr = 0.45
	global turn, laser_running
	if dist > thr:		## actual dist is 0.4 but it takes time to stop (abt 0.5 sec from 0.2 m/s) (min acc = -0.4)
		print("==========Go straight==========")
		vel_msg = vel_msg_init()
		vel_msg.linear.x=linear_vel
		velocity_publisher.publish(vel_msg)
		laser_running = False
	else:
		############################################################################################
		print("!!!!!!!!!     Wall Detected! Stop bot for turn     !!!!!!!!")
		vel_msg = vel_msg_init()
		velocity_publisher.publish(vel_msg)
		############################################################################################
		
		############################################################################################
		print("!!!!!!!!!     Implement Obstacle Avoidance / TURN        !!!!!!!!!!")
		current_angle = 0
		angular_speed = 48*(PI/180)	## 47.5 degrees per second
		
		vel_msg = vel_msg_init()
		if turn == "right":
			relative_angle = 90*(PI/180)
			vel_msg.angular.z = -abs(angular_speed)	##clockwise
		elif turn == "left":
			relative_angle = 90*(PI/180)
			vel_msg.angular.z = abs(angular_speed)	##anti-clockwise
		elif turn == "u":
			relative_angle = 180*(PI/180)
			vel_msg.angular.z = -abs(angular_speed)	##clockwise
		
		if turn == "na":
			print("===========Implement Obstacle Avoidance here===========")	## no turn sign but barruer ahead
			##
		else:
			t0 = rospy.Time.now().to_sec()
			# t0 = time.time()
			print("!!!!!!!!   TURNING BOT   !!!!!!!!")
			while(current_angle < relative_angle):
				velocity_publisher.publish(vel_msg)
				t1 = rospy.Time.now().to_sec()
				# t1 = time.time()
				current_angle = angular_speed*(t1-t0)
		############################################################################################

		############################################################################################
		print("=== (((Obstacle Avoidance / TURN Completed))): Stabilize the Speed and Pose of bot ===")
		vel_msg = vel_msg_init()
		vel_msg.linear.x=linear_vel
		velocity_publisher.publish(vel_msg)
		############################################################################################
		
		############################################################################################
		# while not rospy.is_shutdown():
		# 	vel_msg = vel_msg_init()
		# 	vel_msg.linear.x= linear_vel
		# 	velocity_publisher.publish(vel_msg)
		# 	time.sleep(2)
		# 	break
		############################################################################################

		############################################################################################
		print("!!!!!!  Resuming Default Script !!!!!!")
		laser_running = False	## mark laser thread completion so that the laser_callback can process new requests
		turn = "na"		## reinitilize turn value after successfully completing a turn
		############################################################################################

	

## this subscriber function is called every ~millisecond with the LIDAR laser scan as received by the bot
def laser_callback(msg):
	print("Received laser scan!")
	global laser_running
	# print(msg)
	# print("angle min:", msg.angle_min*180/np.pi, ", angle_max:", msg.angle_max*180/np.pi, ", range_min:", msg.range_min, "range_max:", msg.range_max)
	# print("number of elements in 1 scan:", len(msg.ranges))
	# print(msg.ranges[:-45]+msg.ranges[45:])	## concat 45 to -45 degrees where 0 degree is straight ahead (Note: -ve means to the right)
	print("::::::::range value in front of bot::::::::::::::::::::::::::::::::::::::::::::::::::::::", msg.ranges[0])
	# dist = msg.ranges[0]	## distance of nearest obj in front of bot
	if laser_running == True:
		## Dont process new laser data till the previous process has been completed
		print("Previous laser thread not completed yet")
	else:
		d = msg.ranges
		print("Starting an obstacle handling thread")
		laser_running = True	## Mark the start of a laser process
		thread.start_new_thread(threaded_obstacle_handling, (d,))
		## all time taking process need to be done outside the subscriber function to keep the subscriber and publisher synchronous
		## otherwise the subscriber will resume with old scan values after processing
		


def controller():
	rospy.init_node('hackathonblach', anonymous=True)

	# image_publisher = rospy.Publisher(image_topic, Image)
	## subscribe to image_callback function every time an image is received by the bot
	image_subscriber = rospy.Subscriber(image_topic, Image, image_callback)

	# laser_publisher = rospy.Publisher(laser_topic, LaserScan)
	## subscribe to laser_callback function every time a LIDAR laser scan is received by the bot
	laser_subscriber = rospy.Subscriber(laser_topic, LaserScan, laser_callback)

	# odom_sub = rospy.Subscriber(odometry_topic, Odometry, odom_callback)
	rospy.spin()


def main():
	os.system('gnome-terminal -x ' 'roslaunch hackathon fin.launch')	##open the map with the bot and the world
	# time.sleep(5)
	try:
		controller()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	try:
		image_topic = "/camera/rgb/image_raw"
		laser_topic = "/scan"
		velocity_topic = "/cmd_vel"
		odometry_topic = "/odom"
		velocity_publisher = rospy.Publisher(velocity_topic, Twist, queue_size=10)
		turn = "na"
		laser_running = False
		recognition_running = False
		PI = 3.1415926535897
		linear_vel = 0.2
		main()
	except rospy.ROSInterruptException:
		pass