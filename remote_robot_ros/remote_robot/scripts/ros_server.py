#!/usr/bin/env python

import numpy as np

import rospy
import cv2
from std_msgs.msg import String

from local_communication import LocalComm
from robot_communication import RobotComm


def ros_server():
	global local_comm
	# connect to the django script via localhost
	local_comm = LocalComm(12800)
	local_comm.start()
	print("local comm opened")
	
	# ros node to communicate with the robot
	rospy.init_node('ros_server')
	pub = rospy.Publisher('commands', String, queue_size=10)
	robot0 = RobotComm(0, pub, local_comm, 640, 480)
	local_comm.add_robot(0, robot0, 640, 480)
	
	print("robot0 initialized")

	while not rospy.is_shutdown():
		# do nothing while the other thread is running
		rospy.sleep(0.05)
	
	rospy.spin()



if __name__ == '__main__':
    try:
        ros_server()
    except rospy.ROSInterruptException:
	# this exception is triggered when the application is stopped by CTRL+C
        pass

# shut down the other thread by raising an event
local_comm.stop()
    
