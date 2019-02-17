#!/usr/bin/env python

import socket
import threading
import numpy as np
import time


class LocalComm(threading.Thread):
	"""
	This class is used to communicate with the webserver via a localhost socket
	port (int): the port used by the socket. Must be the same value as the port used by the webserver  
	"""
	def __init__(self, port):
		if type(port) != int:
			raise TypeError("port should be an int")
		if 0 > port or port > 65535:
			raise ValueError('port should be a correct port value')
		threading.Thread.__init__(self)
		self.__end_event = threading.Event()
		self.__PORT = port
		self.__robots = dict()
		self.__connect()
		
	def __connect(self):
		# Connect to the webserver via localhost
		try:
			self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.__socket.settimeout(10)
			self.__socket.connect(('127.0.0.1', self.__PORT))
			self.__socket.sendall(chr(25))
			data = self.__socket.recv(1024)
			print(data)
		except Exception as e:
			print("Could not connect to the local server: " + str(e))
			exit()
		
	def add_robot(self, robot_id, robot, video_width, video_height):
		# Add a robot to the instance container and send info to the webserver
		if type(robot_id) != int:
			raise TypeError("robot_id should be an int")
		if type(video_width) != int or type(video_height) != int:
                        raise TypeError("video_width and video_heigth should be int values")
		self.__robots[robot_id] = robot
		try:
			message = chr(01) + chr(robot_id) + chr(video_height//256) + chr(video_height%256) + chr(video_width//256) + chr(video_width%256)
			self.__socket.sendall(message)
		except:
			pass
		
	def send_video_frame(self, robot_id, frame):
		# send a video frame to the webserver
		if robot_id not in self.__robots:
			raise ValueError("Can't find the provided robot_id")
		try:
			self.__socket.sendall(chr(02) + chr(robot_id))
			h, w, three = frame.shape
			message = frame.tostring()
			self.__socket.sendall(message)	
			time.sleep(0.005)
		except Exception as e:
			print(e)
		
	def __command_callback(self, data):
		# Triggered when a command is received from the webserver
		robot_id = ord(data[1])
		cmd_code = ord(data[2])
		if cmd_code in (0,1,2,3) and robot_id in self.__robots:
			c = {0:'up', 1:'down', 2:'left', 3:'right', 4:'stop'}[cmd_code]
			self.__robots[robot_id].send_command(c)
		
	def run(self):
		# Main thread routine
		# Waits for a message from its socket and tries again if the socket times out (10 sec)
		while not self.__end_event.is_set():
			try:
				data = self.__socket.recv(4096)
				if data:
					header = ord(data[0])
					if header == 10 and len(data) >= 3:
						self.__command_callback(data)
			except socket.timeout:
				pass
			except Exception as e:
				print(e)
				pass

	def stop(self):
		# Stops the thread by raising an event
		self.__end_event.set()











