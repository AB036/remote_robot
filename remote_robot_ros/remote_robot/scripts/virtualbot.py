#!/usr/bin/env python

import numpy as np

import rospy
import cv2
import cv_bridge
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

x,y = 320, 240
cam = np.zeros((480,640,3))
cam[0,0,0] = 255
cam[0,0,1] = 50
cam[0,0,2] = 255
cam[1,1,0] = 255
cam[1,1,1] = 255
command = ""

def callback(data):
	global command
	rospy.loginfo(rospy.get_caller_id() + ' received ' + data.data)
	command = data.data   

if __name__ == '__main__':
	rospy.init_node('virtualbot', anonymous=True)

	rospy.Subscriber('commands', String, callback)

	image_pub = rospy.Publisher("video_frame", CompressedImage, queue_size=1)
	bridge = cv_bridge.CvBridge()

	while not rospy.is_shutdown():
		cam[y-5:y+6,x-5:x+6,:] = np.array([0,0,0])
		if command == "up":
			y -= 3
		elif command == "down":
			y += 3
		elif command == "left":
			x -= 3
		elif command == "right":
			x += 3
		x = max(5, min(x, 634))
		y = max(5, min(y, 474))
		cam[y-5:y+6,x-5:x+6] = np.array([128,255,50])
		#cam[y-5:y+6,x-5:x+6,:] = 255
		
		#print(cam[0][0])
		image_pub.publish(bridge.cv2_to_compressed_imgmsg(cam))
		cv2.imshow("Image window", cam)
		cv2.waitKey(20)
		
		#rospy.sleep(0.02)


