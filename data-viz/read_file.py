
import h5py 
import numpy 
import os
import socket
import struct


index = 0
HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 65432       # The port used by the server
clientserver = None 

def read_file(file_name:str, dataset_name:str):
    file_path = os.path.join("data", f"{file_name}.h5")
    column_position = "/".join([dataset_name, "POSITION"])
    column_orientation = "/".join([dataset_name, "ORIENTATION"])
    global index
    with h5py.File(file_path, 'r') as f:
        data_position = f[column_position][:]
        data_orientation = f[column_orientation][:]
        if index < data_position.shape[0]:
            location = data_position[index]
            quaternion = data_orientation[index]
            index += 1
            return index -1, location, quaternion
        return None, None, None
    return None, None, None
        
def reset_index(): 
    global index 
    index = 0

def reset_index(): 
    global index 
    index = 0

def client_init():
    global clientserver, HOST, PORT, index
    clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    clientsocket.connect((HOST, PORT))
    reset_index()
    
def read_socket(): 
    global clientserver, index
    if clientserver is not None: 
        data = clientserver.recv(4 * 7)  # Each float is 4 bytes, so 7 floats = 28 bytes?
        coord = struct.unpack('7f', data)
        index += 1
        return index-1,coord[:3], coord[3:]
    return None, None, None 
    