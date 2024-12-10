import socket
import struct


class Server:
    def __init__(self, host='127.0.0.1', port=65432):
        self.host = host
        self.port = port
        self.server_socket = None
        self.conn = None 
        self.addr = None
    

    def start(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.conn, self.addr = self.server_socket.accept()

    def send_data(self, data):
        packed_data = struct.pack('7f', *data)
        self.conn.send(packed_data)
        
            

    def close(self):
        self.server_socket.close()

