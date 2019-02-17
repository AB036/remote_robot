import unittest
import socket

from socket_connection import SocketConnection


class TestSocketConnection(unittest.TestCase):
    """TestCase to test the SocketConnection class"""

    def setUp(self):
        """Creates the sockets"""
        self.socket_thread = SocketConnection()  # Creates the socket thread to connect in localhost with ROS
        self.ros_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Simulate a ROS node by creating another socket

    def connect_to_ros(self):
        """Code to connect the sockets"""
        if not self.socket_thread.is_alive():
            self.socket_thread.start()
        self.ros_connection.connect(('localhost', 12800))
        self.ros_connection.send(b'\x19')

    def disconnect_from_ros(self):
        """Code to disconnet and close the sockets"""
        self.ros_connection.close()
        self.socket_thread.stop()

    def test_connection_established(self):
        """Test if the socket can correctly connect and receive the correct message"""
        self.connect_to_ros()
        msg = self.ros_connection.recv(1)
        self.assertEqual(msg, b'\x19')
        self.disconnect_from_ros()

    def test_command_sent_up(self):
        """Test if the socket can send a 'up' command and if the ROS node can receive it"""
        self.connect_to_ros()
        self.ros_connection.recv(1)
        SocketConnection.send_command(0, 0)
        msg = self.ros_connection.recv(3)
        self.assertEqual(msg, b'\x0a' + b'\x00' + b'\x00')
        self.disconnect_from_ros()

    def test_command_sent_down(self):
        """Test if the socket can send a 'down' command and if the ROS node can receive it"""
        self.connect_to_ros()
        self.ros_connection.recv(1)
        SocketConnection.send_command(0, 1)
        msg = self.ros_connection.recv(3)
        self.assertEqual(msg, b'\x0a' + b'\x00' + b'\x01')
        self.disconnect_from_ros()
