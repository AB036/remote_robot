#!/usr/bin/env python

import socket
import threading
import numpy as np
#import cv2

"""
Messages:

hello : header_hello
command : header_cmd | robot_id | cmd_id
video_frame : header_frame | robot_id | h_pixels | w_pixels | data...
"""

class LocalComm(threading.Thread):
	def __init__(self, port):
		self.__end_event = threading.Event()
		self.__PORT = port
		self.__robots = dict()
		self.__connect()
		
	def __connect(self):
		self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.__socket.settimeout(1)
		self.__socket.connect(('127.0.0.1', self.__PORT))
		self.__socket.sendall(chr(25))
		data = self.__socket.recv(1024)
		print(data)
		
	def add_robot(self, robot_id, robot, video_heigth, video_width):
		self.__robots[robot_id] = robot
		message = chr(01) + chr(robot_id) + chr(video_heigth//256) + chr(video_heigth%256) + chr(video_width//256) + chr(video_width%256)
		self.__socket.sendall(message)
		
	def send_video_frame(self, robot_id, frame):
		#cv2.imshow("Robot " + str(robot_id), frame)
		#cv2.waitKey(20)
		self.__socket.sendall(chr(02) + chr(robot_id))
		h, w, three = frame.shape()
		for i in range(h):
			message = chr(03)
			for j in range(w):
				message += chr(frame[i][j][0]) + chr(frame[i][j][1]) + chr(frame[i][j][2])
			self.__socket.sendall(message)
		
	def __command_callback(self, data):
		robot_id = ord(data[1])
		cmd_code = ord(data[2])
		c = {0:'up', 1:'down', 2:'left', 3:'right', 4:'stop'}[cmd_code]
		self.__robots[robot_id].send_command(c)
		
	def run(self):
		while not self.__end_event.is_set():
			try:
				data = self.__socket.recv(4096)
				print("Received: " + data)
				header = ord(data[0])
				if header == 10:
					self.__command_callback(data)
			except socket.timeout:
				pass

	def stop(self):
		self.__end_event.set()











