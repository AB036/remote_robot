import unittest

import socket


from socket_connection import SocketConnection

class TestSocketConnection(unittest.TestCase):
    """TestCase to test the SocketConnection class"""

    def setUp(self):
        pass

    def connect_to_ros(self):
        self.socket_thread = SocketConnection()  # Creates the socket thread to connect in localhost with ROS
        self.ros_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if not self.socket_thread.is_alive():
            self.socket_thread.start()
        self.ros_connection.connect(('localhost', 12800))
        self.ros_connection.send(b'\x19')

    def disconnect_from_ros(self):
        self.socket_thread.stop()
        self.ros_connection.close()

    def test_connection_established(self):
        """The socket can connect"""
        self.connect_to_ros()
        msg = self.ros_connection.recv(1)
        self.assertEqual(msg,b'\x19')
        self.disconnect_from_ros()

    def test_command_sent_up(self):
        """The socket can send a up command"""
        self.connect_to_ros()
        msg = self.ros_connection.recv(1)
        SocketConnection.send_command(0,0)
        msg = self.ros_connection.recv(3)
        self.assertEqual(msg,b'\x0a' + b'\x00' + b'\x00')
        self.disconnect_from_ros()

    def test_command_sent_down(self):
        """The socket can send a down command"""
        self.connect_to_ros()
        msg = self.ros_connection.recv(1)
        SocketConnection.send_command(0,1)
        msg = self.ros_connection.recv(3)
        self.assertEqual(msg,b'\x0a' + b'\x00' + b'\x01')
        self.disconnect_from_ros()




