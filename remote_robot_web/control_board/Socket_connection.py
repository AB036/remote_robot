import socket
from threading import Thread
import numpy as np


class Socket_connection(Thread) :
    """Thread managing the local connection (with sockets) with ROS to send commands and receive video"""

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
            print("Sent command to new robot with id "+str(new_robot_id))

        elif header == b'\x02' : #If received a video frame message
            robot_id = int.from_bytes(self.__client_connection.recv(1), byteorder="big")
            [robot_height,robot_width] = self.__registered_robots[robot_id]
            img = []
            for i in range(robot_width) :
                line = np.array(list(self.__connection.recv(3*robot_width))).reshape(())




    def send_command(self,robot_id,command_id) :
        print("Sent command "+str(command_id)+" to robot "+str(robot_id))
        self.__client_connection.sendall(b'\x0a' + bytes([robot_id]) + bytes([command_id]))
