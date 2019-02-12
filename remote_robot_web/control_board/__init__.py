from control_board.socket_connection import SocketConnection

socket_thread = SocketConnection()  # Creates the socket thread to connect in localhost with ROS
if not socket_thread.is_alive():
    socket_thread.start()