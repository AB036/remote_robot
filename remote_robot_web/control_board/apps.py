import sys
from django.apps import AppConfig
from control_board.socket_connection import SocketConnection


class ControlBoardConfig(AppConfig):
    name = 'control_board'

    @staticmethod
    def _socket_server_should_start():
        """Return whether the thread should start when server is launched."""
        if sys.argv == ['manage.py', 'runserver', '--noreload']:
            return True
        return False

    def ready(self):
        """Starts socket thread on launch."""
        if self._socket_server_should_start():
            socket_thread = SocketConnection()  # Creates the socket thread to connect in localhost with ROS
            socket_thread.start()

