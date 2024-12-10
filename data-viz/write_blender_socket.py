import socket
import struct

HOST = '127.0.0.1'  # Localhost
PORT = 65432        # Port to listen on
serversocket = None
conn = None
addr = None

def server_init(): 
    global HOST, PORT
    serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serversocket.bind((HOST, PORT))
    serversocket.listen(1)
    global conn, addr
    conn, addr = serversocket.accept()
    conn.setblocking(False)

def send_coord(x:float,y:float,z:float,qw:float,qx:float,qy:float,qz:float): 
    global addr, conn
    if conn is not None and addr is not None:
        print(f"Connected by {addr}")
        coords = (x,y,z,qw,qx,qy,qz)
        data = struct.pack('7f', *coords)
        conn.sendall(data)

def server_close(): 
    global conn, serversocket
    if conn is not None and serversocket is not None: 
        conn.close()
        serversocket.close()