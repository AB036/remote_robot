#!/usr/bin/env python

import numpy as np

import rospy
import cv2
from std_msgs.msg import String

from local_communication import LocalComm
from robot_communication import RobotComm


def ros_server():
	
	#connect to the django script via localhost
	local_comm = LocalComm(12345)
	local_comm.start()
	print "local comm opened"
	
	#ros node to communicate with the robot
	rospy.init_node('ros_server')
	pub = rospy.Publisher('commands', String, queue_size=10)
	robot0 = RobotComm(0, pub, local_comm)
	local_comm.add_robot(0, robot0)
	
	print "robot0 initialized"

	while not rospy.is_shutdown():
		rospy.sleep(2)
	
	rospy.spin()



if __name__ == '__main__':
    try:
        ros_server()
    except rospy.ROSInterruptException:
        pass
    
    
