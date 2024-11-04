import numpy as np
import threading as ts
import socket as sock
import time
import struct
from dataStore import DataEntry, DataManager

class DataStream:
    
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self._thread = None
        self._done = False

    def start(self, type: str, callback = None) -> None:
            self._thread = ts.Thread(target=self.streamThread, name='streamThread')
            self._callback = callback
            self._thread.start()
            
    def terminate(self, ) -> None:
        self._done = True
        if self._thread is not None:
            self._thread.join()
            self._thread = None
    
    def streamThread(self,):
        pass

    def quaternionDataStreamThread(self, ):
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

    def swipeDataStreamThread(self, ):
        with sock.socket(sock.AF_INET, sock.SOCK_STREAM) as s:
            s.connect((self.host, self.port))
            s.sendall(b'swipe?\n')
            print("connected")

            while not self._done:
                # read start bytes
                swipeState = s.recv(1).decode()
                if swipeState not in 'udi': continue

                if swipeState in 'ud':
                    s.sendall(b'ack\n')
                    self._callback(swipeState)
                
    def dataCollectionStreamThread(self, ):
        # data format
        # struct DataEntry { // size 128bytes: 117 + 11 alignment padding
        #     unsigned long microsT; // 4 bytes + 4 padding
        #     double accelx, accely, accelz; // 8 bytes * 3
        #     double gyrox, gyroy, gyroz; // 8 bytes * 3
        #     double magnx, magny, magnz; // 8 bytes * 3
        #     double roll, pitch, yaw; // 8 bytes * 3
        #     int8_t tempbno;  // 1 byte + 7 padding
        #     double tempbmp; // 8 bytes
        #     double pressure; // 8 bytes 
        # };

        X = 0
        Y = 1
        Z = 2    
        with DataManager('test.csv') as dm:
            with sock.socket(sock.AF_INET, sock.SOCK_STREAM) as s:
                s.settimeout(11)
                s.connect((self.host, self.port))
                s.sendall(b'RDY\n')
                print("connected")
                d = DataEntry()
                while not self._done:
                    try:
                        raw_data = s.recv(128)
                        data_entry_struct = struct.unpack('<LLddddddddddddBBBBLdd', raw_data) # manually padding
                        d.ts = data_entry_struct[0]
                        d.accel[X] = data_entry_struct[2] # skip bc of padding see struct definition
                        d.accel[Y] = data_entry_struct[3]
                        d.accel[Z] = data_entry_struct[4]
                        d.gyro[X] = data_entry_struct[5]
                        d.gyro[Y] = data_entry_struct[6]
                        d.gyro[Z] = data_entry_struct[7]
                        d.magn[X] = data_entry_struct[8]
                        d.magn[Y] = data_entry_struct[9]
                        d.magn[Z] = data_entry_struct[10]
                        d.rpy[X] = data_entry_struct[11]
                        d.rpy[Y] = data_entry_struct[12]
                        d.rpy[Z] = data_entry_struct[13]
                        d.tempbno = data_entry_struct[14]
                        d.tempbmp = data_entry_struct[19] # padding skip
                        d.pressure = data_entry_struct[20]              
                        
                        dm.write(d)
                        time.sleep(0.01)
                    except:
                        pass

    def rttTest(self, ):
        with sock.socket(sock.AF_INET, sock.SOCK_STREAM) as s:
            s.connect((self.host, self.port))
            s.send(b'rr')
            print('connected')
            data = s.recv(1)
            s.sendall(data)


