import socket

def start_connection() :
    host = '127.0.0.1'
    port = 12800

    connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connection.bind((host, port))
    connection.listen(5)
    client_connection, info = connection.accept()

    received_msg = client_connection.recv(1)
    if received_msg == b'\x19' :
        print("Local connection established")
        client_connection.send(b'\x19')
    return connection,client_connection

def stop_connection(connection,client_connection) :
    print("Closing connection")
    client_connection.close()
    connection.close()

def send_command(client_connection) :
    client_connection.send(b'\x0a')
    client_connection.send(b'\x00')

start_connection()