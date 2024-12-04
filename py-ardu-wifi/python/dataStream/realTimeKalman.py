import socket as sock
import struct
from dataStore import WifiDataEntry, WifiDataManager, DataEntry, DataManager
from dataStream.dataStream import DataStream
import numpy as np
from model.spiral_model import *
from scipy.spatial.transform import Rotation

import sys
import os

# Navigate two levels up and add the path
base_path = os.path.abspath(os.path.join(__file__, "../../data/filters"))
sys.path.append(base_path)

# Import the file
from HelixonKalmanFilter import *

# Dwifi: Best Combination: P=0.01, Q=0.05, R=1, H=1
# NormalUDP1: Best Combination: P=0.01, Q=0.01, R=1, H=1 (GTE = 14)
# NormalUDP1: 
# RandomUP1: Best Combination: P=0.01, Q=1, R=0.01, H=1 (GTE = 0.89)
# RandomUP2: Best Combination: P=0.01, Q=0.01, R=1, H=1 (GTE = 13)
# RandomUP3: Best Combination: P=0.01, Q=0.01, R=1, H=1 (GTE = 5)
# RandomUP4: Best Combination: P=0.01, Q=0.01, R=1, H=1 (GTE = 16)
# RandomUP5: Best Combination: P=0.01, Q=0.01, R=1, H=1 (GTE = 7)
# RandomUP6: Best Combination: P=0.01, Q=1, R=0.01, H=1 (GTE = 5)

# P (measurement cov mat)
P = np.identity(2) * .01
# Q (process noise)
Q = np.identity(2) * 10
# R (measurement noise)
R = np.identity(1) * 0.001
# H (measurement matrix)
H = np.array([
    [ 1., 0. ], 
])

# function to get A (state transition matrix) for certain dt
def getA(dt: float):
    return np.array([
        [1., dt],  # Height
        [0., 1.]   # Velocity
    ])

# function to get B (control transition matrix) for certain dt
def getB(dt: float):
    return np.array([
        [0.5 * dt**2],  # Height
        [dt]            # Velocity
    ])

# creating spiral model
spiral_pitch = 4 #m
spiral_radius = 8 #m
path_width = 2.4 #m
Spiral = Spiral(spiral_pitch, spiral_radius, path_width)

class UDPDataStream(DataStream):

    def streamThread(self,):
        X = 0
        Y = 1
        Z = 2

        with sock.socket(sock.AF_INET, sock.SOCK_DGRAM) as s:  # UDP socket
            s.settimeout(5)  # Set timeout for receiving data
            print("Ready to send/receive data")
            with WifiDataManager('wifi.csv') as wdm:
                with DataManager('raw.csv') as dm:
                    # Send start command
                    s.sendto(b'strt\n', (self.host, self.port))
                    print("Sent start command to Arduino.")
                    while not self._done:
                        # try:
                            # Receive packet of 257 bytes
                            packet, addr = s.recvfrom(257)
                            packet_type = packet[0:1].decode('utf-8')  # First byte determines type

                            if packet_type == 'w':  # Wi-Fi data
                                wifid = WifiDataEntry()
                                raw_wifi_data = packet[1:]  # Remaining 256 bytes
                                data_entry_struct = struct.unpack('<Lb' + '6B'*25 + 'x' + 'i'*25, raw_wifi_data)
                                wifid.ts = data_entry_struct[0]
                                wifid.rssiCnt = data_entry_struct[1]
                                for i in range(wifid.rssiCnt):
                                    bssid_offset = 2
                                    rssi_offset = bssid_offset + 6*25
                                    bssid = data_entry_struct[bssid_offset + 6*i:bssid_offset + 6*(i+1)]
                                    rssi = data_entry_struct[rssi_offset + i]
                                    wifid.addData(bssid, rssi)

                                for i, orientation in enumerate(raw_rpy):
    
                                    # check if quat or euler
                                    if orientation.shape[-1] == 4:
                                        rot = Rotation.from_quat(orientation).inv()
                                    else:
                                        rot = Rotation.from_euler('xyz', orientation, degrees=True).inv()
                                    accel[i] = rot.apply(araw[i])

                                    global_accel_z = accel[:,Z].reshape(N, 1, 1)

                                wdm.write(wifid)


                            elif packet_type == 'r':  # Raw data
                                raw_data = packet[1:]  # Remaining 256 bytes
                                # Split into two 128-byte packets
                                for i in range(2):
                                    chunk = raw_data[i*128:(i+1)*128]
                                    data_entry_struct = struct.unpack('<L4x3d3d3d3dB7xdd', chunk)
                                    d = DataEntry()
                                    d.ts            = data_entry_struct[0]
                                    d.linaccel[X]   = data_entry_struct[1]
                                    d.linaccel[Y]   = data_entry_struct[2]
                                    d.linaccel[Z]   = data_entry_struct[3]
                                    d.gyro[X]       = data_entry_struct[4]
                                    d.gyro[Y]       = data_entry_struct[5]
                                    d.gyro[Z]       = data_entry_struct[6]
                                    d.magn[X]       = data_entry_struct[7]
                                    d.magn[Y]       = data_entry_struct[8]
                                    d.magn[Z]       = data_entry_struct[9]
                                    d.rpy[X]        = data_entry_struct[10]
                                    d.rpy[Y]        = data_entry_struct[11]
                                    d.rpy[Z]        = data_entry_struct[12]
                                    d.tempbno       = data_entry_struct[13]
                                    d.tempbmp       = data_entry_struct[14]
                                    d.pressure      = data_entry_struct[15]
                                    dm.write(d)

                        # except sock.timeout:
                        #     print("Socket timeout, no data received.")

                    # Send stop command
                    s.sendto(b'stop\n', (self.host, self.port))
                    print("Sent stop command to Arduino.")

