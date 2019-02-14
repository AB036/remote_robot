#!/usr/bin/env python

import numpy as np
import time

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import cv2
import cv_bridge

class RobotComm:
	def __init__(self, robot_id, ros_publisher, local_comm, video_width, video_heigth):
		if type(robot_id) != int:
			raise TypeError('robot_id should be an int')
		if type(video_width) != int:
			raise TypeError('video_width should be an int')
		if type(video_heigth) != int:
			raise TypeError('video_heigth should be an int')
		self.__robot_id = robot_id
		self.__pub = ros_publisher
		self.__comm = local_comm
		rospy.Subscriber('video_frame', CompressedImage, self.__video_callback)
		self.__bridge = cv_bridge.CvBridge()
		self.__w = video_width
		self.__h = video_heigth
		
	def __video_callback(self, data):
		try:
			cv_image = self.__bridge.compressed_imgmsg_to_cv2(data, "bgr8")
			self.__comm.send_video_frame(self.__robot_id, cv_image)
			#print(time.time())
		except cv_bridge.CvBridgeError as e:
			print(e)
		
	def send_command(self, command):
		rospy.loginfo("Sending to robot " + str(self.__robot_id) + ": " + command)
		self.__pub.publish(command)
