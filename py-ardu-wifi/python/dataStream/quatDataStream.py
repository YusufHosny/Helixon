import numpy as np
import socket as sock
import struct
from dataStream.dataStream import DataStream
from visualizer import Visualizer

class QuatDataStream(DataStream):
    
    def __init__(self,):
        super.__init__()
        self.viz = Visualizer()
        self._callback = self.viz.quatcallback
   
    def streamThread(self,):
        with sock.socket(sock.AF_INET, sock.SOCK_STREAM) as s:
            s.connect((self.host, self.port))
            s.sendall(b'ready\n')
            print("connected")

            while not self._done:
                quat = np.array([0., 0., 0., 0.])
                # read start bytes
                raw_data = s.recv(4)
                if raw_data != b'strt': continue

                for i in range(4):
                    # read data from socket
                    raw_data = s.recv(8)

                    # if data recvd is invalid skip
                    if len(raw_data) != 8: 
                        quat = self.quat
                        break

                    # unpack binary data as big endian double
                    dbl_data = struct.unpack('<d', raw_data)
                    quat[i] = dbl_data[0]

                # use quat
                self._callback(quat)
