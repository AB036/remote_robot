import socket
from threading import Thread

class SocketConnection(Thread) :

    def __init__(self):
        Thread.__init__(self)
        self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def run(self):
        self.__start_connection()
        while True :
            self.__listen()

    def __start_connection(self) :
        host = '127.0.0.1'
        port = 12800

        self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection.bind((host, port))
        self.connection.listen(5)
        self.client_connection, info = self.connection.accept()

        received_msg = self.client_connection.recv(1)
        if received_msg == b'\x19' :
            print("Local connection established")
            self.client_connection.send(b'\x19')
        return self.connection,self.client_connection

    def __stop_connection(self) :
        print("Closing connection")
        self.client_connection.close()
        self.connection.close()

    def __listen(self) :
        header = self.client_connection.recv(1)
        if header == b'\x01' : #If received a new robot message
            new_robot_id = int.from_bytes(self.client_connection.recv(1),byteorder="big")
            new_robot_height = int.from_bytes(self.client_connection.recv(2),byteorder="big")
            new_robot_width = int.from_bytes(self.client_connection.recv(2), byteorder="big")
            self.__send_command(new_robot_id)
            #addrobot(new_robot_id,new_robot_height,new_robot_width)
        elif header == b'\x02' : #If received a video frame message
            ...

    def __send_command(self,robot_id) :
        self.client_connection.send(b'\x0a')
        self.client_connection.send(robot_id.to_bytes(1,byteorder="big"))
        self.client_connection.send(b'\x00')

socket_thread = SocketConnection()
socket_thread.start()