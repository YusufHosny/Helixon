import socket as sock
import struct
from dataStore import WifiDataEntry, WifiDataManager, DataEntry, DataManager
from dataStream.dataStream import DataStream
import numpy as np
from scipy.spatial.transform import Rotation
from hlxon_hdf5io import *
from filters.HelixonKalmanFilter import *
from model.spiral_model import *

from pickle import load

with open(os.path.join("model", "wifi_model.pkl"), "rb") as f:
    best_rf = load(f)

with open(os.path.join("model", "bssid_map.pkl"), "rb") as f:
    bssidMap = load(f)

raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure1, wifidata, gt_timestamp, gt_position, gt_orientation = readHDF5('RandomUDP1')
raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure2, wifidata, gt_timestamp, gt_position, gt_orientation = readHDF5('RandomUDP2')
raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure3, wifidata, gt_timestamp, gt_position, gt_orientation = readHDF5('RandomUDP3')
raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure4, wifidata, gt_timestamp, gt_position, gt_orientation = readHDF5('RandomUDP4')
raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure5, wifidata, gt_timestamp, gt_position, gt_orientation = readHDF5('RandomUDP5')
raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure6, wifidata, gt_timestamp, gt_position, gt_orientation = readHDF5('RandomUDP6')

p0_1 = np.mean(raw_pressure1[:200])
p0_2 = np.mean(raw_pressure2[:200])
p0_3 = np.mean(raw_pressure3[:200])
p0_4 = np.mean(raw_pressure4[:200])
p0_5 = np.mean(raw_pressure5[:200])
p0_6 = np.mean(raw_pressure6[:200])

alpha = 1.16e-4
p0 = np.mean(p0_1, p0_2, p0_3, p0_4, p0_5, p0_6)

# P (measurement cov mat)
P = np.identity(2) * .0001
# Q (process noise)
Q = np.identity(2) * .01

# R (measurement noise)
R_PRESSURE = np.diag([700., 700., 10.])
R_WIFI = np.diag([700., 700., 10000.])

# H (measurement matrix)
H_PRESSURE = np.array([
    [1., 0., 0., 0., 0., 0.],
    [0., 1., 0., 0., 0., 0.], 
    [0., 0., 1., 0., 0., 0.], 
])
H_WIFI = np.array([
    [1., 0., 0., 0., 0., 0.],
    [0., 1., 0., 0., 0., 0.], 
    [0., 0., 1., 0., 0., 0.], 
])


# function to get A (state transition matrix) for certain dt
def getA(dt: float):
    return np.array([
# positions ||   velos   |   
[1., 0., 0.,  dt, 0., 0. ],  # px
[0., 1., 0.,  0., dt, 0. ],  # py 
[0., 0., 1.,  0., 0., dt ],  # pz
[0., 0., 0.,  1., 0., 0. ],  # vx
[0., 0., 0.,  0., 1., 0. ],  # vy
[0., 0., 0.,  0., 0., 1. ],  # vz
    ])

# function to get B (control transition matrix) for certain dt
def getB(dt: float):
    return np.array([
#               accels              |
[.5*dt**2,  0.,         0.         ],  # px
[0.,        .5*dt**2,   0.         ],  # py 
[0.,        0.,         .5*dt**2   ],  # pz 
[dt,        0.,         0.         ],  # vx
[0.,        dt,         0.         ],  # vy
[0.,        0.,         dt         ],  # vz
    ])

# Initializing Kalman Filter
kf = HelixonKalmanFilter(getA, getB, P, Q)

# Creating spiral model
spiral_pitch = 4.1 #m
spiral_radius = 8 #m
path_width = 2.4 #m
spiral = Spiral(spiral_pitch, spiral_radius, path_width)


class UDPDataStream(DataStream):

    def streamThread(self,):
        X = 0
        Y = 1
        Z = 2

        with sock.socket(sock.AF_INET, sock.SOCK_DGRAM) as s:  # UDP socket
            s.settimeout(5)  # Set timeout for receiving data
            print("Ready to send/receive data")

            # Send start command
            s.sendto(b'strt\n', (self.host, self.port))
            print("Sent start command to Arduino.")
            while not self._done:
                    
                # Receive packet of 257 bytes
                packet, addr = s.recvfrom(257)
                packet_type = packet[0:1].decode('utf-8')  # First byte determines type

                if packet_type == 'w':  # Wi-Fi data
                    print("") # Do nothing

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

                        ### Kalman filter implementation
                        ts = d.ts * 1e-6  
                        accel = np.array([d.linaccel[X], d.linaccel[Y], d.linaccel[Z]])
                        orientation = [d.rpy[X], d.rpy[Y], d.rpy[Z]]

                        # Acceleration into global coords
                        rot = Rotation.from_euler('xyz', orientation, degrees=True).inv()
                        accel_global = rot.apply(accel)

                        # Predict 
                        dt = ts - current_time
                        current_time = ts
                        kf.predict(accel_global.reshape((3, 1)), dt)

                        # Update 
                        height = np.log(d.pressure / p0) / -alpha
                        state = kf.update(spiral.point_at_z(height).reshape((3, 1)), H_PRESSURE, R_PRESSURE)
                        print(state)

                    # except sock.timeout:
                    #     print("Socket timeout, no data received.")

            # Send stop command
            s.sendto(b'stop\n', (self.host, self.port))
            print("Sent stop command to Arduino.")




