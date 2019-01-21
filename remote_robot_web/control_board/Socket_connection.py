import socket
from threading import Thread
import numpy as np

import cv2


class SocketReadingException(Exception):
    pass

class Socket_connection(Thread) :
    """Thread managing the local connection (with sockets) with ROS to send commands and receive video"""

    frame = np.zeros((480,640,3))
    #frame = cv2.imread("C:/image.jpg")
    #np.zeros((480,640,3))
    print(frame.shape)

    def __init__(self):
        Thread.__init__(self)
        self.__connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__registered_robots = dict() #Dict containing the info of each registered robots (key = id, value = [height,width])

    def run(self):
        self.__start_connection()
        while True :
            self.__listen()

    def __start_connection(self) :
        print("Establishing connection...")
        host = '0.0.0.0'
        port = 12800

        self.__connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__connection.bind((host, port))
        self.__connection.listen(5)
        self.__client_connection, info = self.__connection.accept()

        received_msg = self.__client_connection.recv(1)
        print(received_msg)
        if received_msg == b'\x19' : #If received a reply from the client after establishing connection
            print("Local connection established")
            self.__client_connection.send(b'\x19')

    def __stop_connection(self) :
        print("Closing connection")
        self.__client_connection.close()
        self.__connection.close()

    def __listen(self) :
        """Listen to the messages sent by ROS"""
        header = self.__client_connection.recv(1) #Read the first byte to get the type of the request

        if header == b'\x01' : #If received a new robot message
            new_robot_id = self.__client_connection.recv(1)[0]
            new_robot_height = int.from_bytes(self.__client_connection.recv(2), byteorder="big")
            new_robot_width = int.from_bytes(self.__client_connection.recv(2), byteorder="big")
            self.__registered_robots[new_robot_id] = [new_robot_height,new_robot_width]
            print("Added new robot with id "+str(new_robot_id)+" and dim "+str(new_robot_height)+","+str(new_robot_width))

        elif header == b'\x02' : #If received a video frame message
            robot_id = int.from_bytes(self.__client_connection.recv(1), byteorder="big")
            [robot_height,robot_width] = self.__registered_robots[robot_id]
            print("Starting to receive an image from robot {} with res {} x {}".format(robot_id,robot_height,robot_width))
            img = []
            #Starting reading each line of the image
            for i in range(robot_height) :
                second_header = self.__client_connection.recv(1)
                if second_header != b'\x03' :
                    raise SocketReadingException("Error while reading lines from camera image") ;
                line = []
                for j in range(robot_width) :
                    line.append(list(self.__client_connection.recv(3)))
                img.append(line)
            Socket_connection.frame = np.array(img)
            print(Socket_connection.frame.shape)

    def send_command(self,robot_id,command_id) :
        print("Sent command "+str(command_id)+" to robot "+str(robot_id))
        self.__client_connection.sendall(b'\x0a' + bytes([robot_id]) + bytes([command_id]))
