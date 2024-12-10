import socket as sock
import struct
from dataStore import WifiDataEntry, DataEntry, DataManager
from dataStream.dataStream import DataStream
import numpy as np
from scipy.spatial.transform import Rotation
from hlxon_hdf5io import *
from filters.HelixonKalmanFilter import *
from model.spiral_model import *
from socket_server import Server


alpha = 1.16e-4

# P (measurement cov mat)
P = np.identity(6) * .0001
# Q (process noise)
Q = np.identity(6) * .1

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

p_data = []

serv = Server()
serv.start()

#serv.close()
class UDPDataStream(DataStream):

    def streamThread(self,):
        X = 0
        Y = 1
        Z = 2

        with sock.socket(sock.AF_INET, sock.SOCK_DGRAM) as s:  # UDP socket
            current_time = 0

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
                    time.sleep(.01)
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

                        if len(p_data) < 200:
                            p_data.append(d.pressure)
                            print("Added")
                        elif (len(p_data) == 200):
                            p_data.append(d.pressure)
                            p0 = np.mean(p_data)
                            print("computed mean")
                        else:
                            print("Printing")
                            ### Kalman filter implementation
                            ts = d.ts * 1e-6  
                            accel = np.array([d.linaccel[X], d.linaccel[Y], d.linaccel[Z]])
                            orientation = [d.rpy[X], d.rpy[Y], d.rpy[Z]]

                            dt = ts - current_time
                            current_time = ts
                            kf.predict(np.array(accel).reshape((3, 1)), dt)
                            
                            # Update 
                            height = np.array(np.log(d.pressure / p0) / -alpha).reshape(1,1)
                            kf.update(spiral.point_at_z(height).reshape((3, 1)), H_PRESSURE, R_PRESSURE)
                            pos = kf.xhat.flatten()[:3]
                            print(f'p kalman: {pos}')
                            print(f'p raw: {spiral.point_at_z(height.flatten()).reshape((3, 1))}')
                            quat = Rotation.from_euler('xyz', orientation, degrees=True).as_quat(scalar_first=True)

                            serv.send_data((*pos, *quat))

                    # except sock.timeout:
                    #     print("Socket timeout, no data received.")

            # Send stop command
            s.sendto(b'stop\n', (self.host, self.port))
            print("Sent stop command to Arduino.")




