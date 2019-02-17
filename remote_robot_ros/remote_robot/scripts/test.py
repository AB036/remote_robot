#!/usr/bin/env python

import numpy as np
import socket
import time
import unittest

import rospy
import cv2
from std_msgs.msg import String



class TestRosServer(unittest.TestCase):

	def test_ros_server(self):
		# A monolithic test to ensure the different steps run in the correct order
		# (i.e. test the set-up and then test the execution)
		global command
		global conn
		
		# The program sends 0x19 when it connects
		data = conn.recv(1)
		conn.sendall('\x19')
		self.assertEqual(data, '\x19')
		
	#def test_add_robot(self):
		# The program adds a robot with the correct values when it connects
		data = conn.recv(6)
		self.assertEqual(data, '\x01\x00\x01\xe0\x02\x80')

	#def test_command(self):
		# The program sends the command to the robot when it receives one from the server
		conn.sendall("\x0a\x00\x03")
		time.sleep(1)
		self.assertEqual(command, 'right')
		
	#def test_bad_command(self):
		# The program does not send a new command if the command code is incorrect
		conn.sendall("\x0a\x00\xf8") # 0xf8 is not a valid command code
		time.sleep(1)
		self.assertEqual(command, 'right')




command = ""

def callback(data):
	global command
	print(data.data)
	rospy.loginfo(rospy.get_caller_id() + ' received ' + data.data)
	command = data.data  


if __name__ == '__main__':
    try:
		rospy.init_node('testbot', anonymous=True)
		rospy.Subscriber('commands', String, callback)
		
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.bind(('', 12800))
		s.listen(1)
		conn, addr = s.accept()
		
		unittest.main()
		
		conn.close()
		s.close()
		
    except rospy.ROSInterruptException:
        pass


local_comm.stop()
    
