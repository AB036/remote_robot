import socket
from threading import Thread
import numpy as np
from time import sleep


class SocketReadingException(Exception):
    pass


class RobotCommand:
    """Class representing a command to send to a robot."""

    def __init__(self, robot_id, command_id):
        self.__robot_id = robot_id
        self.__command_id = command_id

    def get_bytes_command(self):
        """Converts the command into bytes to send with the socket"""
        return b'\x0a' + bytes([self.__robot_id]) + bytes([self.__command_id])

    def get_string_command(self):
        """Converts the command into a string message"""
        return "Sent command " + str(self.__command_id) + " to robot " + str(self.__robot_id)


class SocketConnection(Thread):
    """Thread managing the local connection (with sockets) with ROS to send commands and receive video"""

    # frame = np.random.randint(0,255,(480,640,3))
    frame = np.zeros((480, 640, 3))  # Static variable containing the currently received frame
    command = None  # Static variable containing the currently sent command

    def __init__(self):
        Thread.__init__(self)
        self.__connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__registered_robots = dict()  # Dict containing the info of each registered robots (key = id, value = [height,width])

    def run(self):
        self.__start_connection()
        while True:
            # Listen to receive messages
            self.__listen()

            # Write messages if a command needs to be sent
            if SocketConnection.command is not None:
                print(SocketConnection.command.get_string_command())
                self.__client_connection.sendall(SocketConnection.command.get_bytes_command())
                SocketConnection.command = None

            sleep(0.001)

    def __start_connection(self):
        print("Establishing connection...")
        host = '0.0.0.0'
        port = 12800

        self.__connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.__connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__connection.bind((host, port))
        self.__connection.listen(5)
        self.__client_connection, info = self.__connection.accept()

        received_msg = self.__client_connection.recv(1)
        print(received_msg)
        if received_msg == b'\x19':  # If received a reply from the client after establishing connection
            print("Local connection established")
            self.__client_connection.send(b'\x19')

    def __stop_connection(self):
        print("Closing connection")
        self.__client_connection.close()
        self.__connection.close()

    def __listen(self):
        """Listen to the messages sent by ROS"""
        header = self.__client_connection.recv(1)  # Read the first byte to get the type of the request
        # print(header)

        if header == b'\x01':  # If received a new robot message
            new_robot_id = self.__client_connection.recv(1)[0]
            new_robot_height = int.from_bytes(self.__client_connection.recv(2), byteorder="big")
            new_robot_width = int.from_bytes(self.__client_connection.recv(2), byteorder="big")
            self.__registered_robots[new_robot_id] = (new_robot_height, new_robot_width)
            print("Added new robot with id " + str(new_robot_id) + " and dim " + str(new_robot_height) + "," + str(
                new_robot_width))

        elif header == b'\x02':  # If received a video frame message
            robot_id = int.from_bytes(self.__client_connection.recv(1), byteorder="big")
            robot_height, robot_width = self.__registered_robots[robot_id]

            img_as_byte = b''
            # Reads data while image is not full
            while len(img_as_byte) < robot_height * robot_width * 3:
                img_as_byte += self.__client_connection.recv(2 ** 20)
            if len(img_as_byte) > robot_height * robot_width * 3:
                img_as_byte = img_as_byte[:(robot_height * robot_width * 3)]
            img = np.frombuffer(img_as_byte, dtype=np.uint8).reshape((robot_height, robot_width, 3))
            SocketConnection.frame = img.copy()

    @staticmethod
    def send_command(robot_id, command_id):
        """Tells the thread to send a command by switching the static variable command"""
        SocketConnection.command = RobotCommand(robot_id, command_id)
