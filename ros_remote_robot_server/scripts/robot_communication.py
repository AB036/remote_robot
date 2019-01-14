#!/usr/bin/env python

import threading
import numpy as np

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import cv2
import cv_bridge

class RobotComm(threading.Thread):
	def __init__(self, robot_id, ros_publisher, local_comm, video_heigth, video_width):
		self.__robot_id = robot_id
		self.__pub = ros_publisher
		self.__comm = local_comm
		rospy.Subscriber('video_frame', CompressedImage, self.__video_callback)
		self.__bridge = cv_bridge.CvBridge()
		self.__w = video_width
		self.__h = video_heigth
		
	def __video_callback(self, data):
		cv_image = self.__bridge.compressed_imgmsg_to_cv2(data, "bgr8")
		self.__comm.send_video_frame(self.__robot_id, cv_image)
		
	def send_command(self, command):
		rospy.loginfo("Sending to robot " + str(self.__robot_id) + ": " + command)
		self.__pub.publish(command)
		
	def run(self):
		pass
