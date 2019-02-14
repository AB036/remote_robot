#!/usr/bin/env python

import socket
import threading
import numpy as np
import time


class LocalComm(threading.Thread):
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
		try:
			self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.__socket.settimeout(10)
			#self.__socket.connect(('138.195.207.184', self.__PORT))
			self.__socket.connect(('127.0.0.1', self.__PORT))
			self.__socket.sendall(chr(25))
			data = self.__socket.recv(1024)
			print(data)
		except Exception as e:
			print("Could not connect to the local server: " + str(e))
			exit()
		
	def add_robot(self, robot_id, robot, video_width, video_heigth):
		self.__robots[robot_id] = robot
		try:
			message = chr(01) + chr(robot_id) + chr(video_heigth//256) + chr(video_heigth%256) + chr(video_width//256) + chr(video_width%256)
			self.__socket.sendall(message)
		except:
			pass
		
	def send_video_frame(self, robot_id, frame):
		try:
			self.__socket.sendall(chr(02) + chr(robot_id))
			h, w, three = frame.shape
			message = frame.tostring()
			self.__socket.sendall(message)	
			time.sleep(0.005)
		except Exception as e:
			print(e)
		
	def __command_callback(self, data):
		robot_id = ord(data[1])
		cmd_code = ord(data[2])
		c = {0:'up', 1:'down', 2:'left', 3:'right', 4:'stop'}[cmd_code]
		self.__robots[robot_id].send_command(c)
		
	def run(self):
		while not self.__end_event.is_set():
			try:
				data = self.__socket.recv(4096)
				#print("Received: " + data)
				if data:
					header = ord(data[0])
					if header == 10:
						self.__command_callback(data)
			except socket.timeout:
				pass
			except Exception as e:
				print(e)
				pass

	def stop(self):
		self.__end_event.set()











