"""
Kalman Filter using IMU and WiFi data
Additionally low pass filters outputs and incorporates floorplan data
"""

import numpy as np
import matplotlib.pyplot as plt
from hlxon_hdf5io import *
from scipy.spatial.transform import Rotation
from filters.HelixonKalmanFilter import *
from metrics import *
import time

"""
Load data
"""
print('Loading data...', end='')
# get data from hdf5
raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure, wifidata, gt_timestamp, gt_position, gt_orientation = readHDF5('RandomUDP6')
print('Done.')


"""
Preprocess some data
"""
print('Processing data...', end='')
# convenience
X, Y, Z = 0, 1, 2
N = len(raw_timestamp)

# process wifi data
wifi_timestamp = np.array([row[0] for row in wifidata])*1e-6
wifi_cnts = np.array([row[1] for row in wifidata])
bssids = np.array([[row[i].decode() for i in range(2, len(row), 2)] for row in wifidata])
rssis = np.array([[row[i] for i in range(3, len(row), 2)]  for row in wifidata])

# get sensor data
araw = np.array(raw_9dof[:, :3])
gyro = np.array(raw_9dof[:, 3:6])
magn = np.array(raw_9dof[:, 6:])
pres = np.array(raw_pressure).reshape((-1, 1))
alpha = 1.16e-4
ts = np.array(raw_timestamp)*1e-6
gt_timestamp = np.array(gt_timestamp)*1e-6

# p0 is first real pressure measurement
p0 = np.mean(raw_pressure[:200])
pres[np.where(pres > p0)] = p0

# remove offsets gt pos
offset =  gt_position[0]
gt_position = np.array(gt_position)
gt_position -= offset

# rotate acceleration to global coords
accel = np.zeros_like(araw)
for i, orientation in enumerate(raw_rpy):
    
    # check if quat or euler
    if orientation.shape[-1] == 4:
        rot = Rotation.from_quat(orientation).inv()
    else:
        rot = Rotation.from_euler('xyz', orientation, degrees=True).inv()
    accel[i] = rot.apply(araw[i])
print('Done.')

"""
Kalman Filter Definition

State Vector

format:
    - [0:3] positions x y z (meters global coords)
    - [0:6] velocities x y z (m/s global coords)
"""

# P (measurement cov mat)
P = np.identity(6) * .01
# Q (process noise)
Q = np.identity(6) * 30
# R (measurement noise)
R_PRESSURE = np.diag([10., 10., 1.]) * 1
R_WIFI = np.diag([1., 1., 100.]) * 10000
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


kf = HelixonKalmanFilter(getA, getB, P, Q)

"""
Load Wifi Random Forest Regression Model
"""
from pickle import load
with open(os.path.join("model", "wifi_model.pkl"), "rb") as f:
    best_rf = load(f)

"""
Load BSSID mapping
"""
with open(os.path.join("model", "bssid_map.pkl"), "rb") as f:
    bssidMap = load(f)

"""
Spiral model
"""
from model.spiral_model import Spiral
spiral_pitch = 4.1 #m
spiral_radius = 8 #m
path_width = 2.4 #m
spiral = Spiral(spiral_pitch, spiral_radius, path_width)
spiral.align_to_spiral(gt_position)
spiral.phase = 0

"""
hasNext and getNext functions to iterate over all data (imu and wifi) ordered by their timestamps
"""
ix_raw = 1
ix_wifi = 1
def hasNext():
    return ix_raw < len(ts) or ix_wifi < len(wifi_timestamp)

def getNext():
    global ix_raw 
    global ix_wifi
    
    # decide what to output next based on their timestamps
    WIFINEXT = 1
    RAWNEXT = 0
    if ix_raw >= len(ts):
        nexttype = WIFINEXT
    elif ix_wifi >= len(wifi_timestamp):
        nexttype = RAWNEXT
    else:
        nexttype = np.argmin([ts[ix_raw], wifi_timestamp[ix_wifi]])

    # output data
    if nexttype == RAWNEXT: # raw
        ix_raw += 1
        ti = ts[ix_raw-1]
        dt = ti-ts[ix_raw-2]
        return 'raw', (accel[ix_raw-1, :3], pres[ix_raw-1], dt, ti)
    
    elif nexttype == WIFINEXT: # wifi
        ix_wifi += 1
        ti = wifi_timestamp[ix_wifi-1]

        # map bssid data to model format
        dout = np.ones((1, bssidMap.shape[0])) * -100
        for bssid, rssi in zip(bssids[ix_wifi-1], rssis[ix_wifi-1]):
            if bssid in bssidMap:
                ix = np.argmax(bssidMap == bssid)
                dout[0, ix] *= 0
                dout[0, ix] += rssi

        return 'wifi',  (dout, ti)

"""
Execute Kalman filter
"""
print('Running Kalman Filter...', end='')
pos = []
times = []
heights = []
pwifi = []
while hasNext():
    datatype, data = getNext()
    
    if datatype == 'raw':
        # reorganize data
        data, presval, dt, ti = data

        # predict step
        data = np.array(data).reshape((3, 1))
        kf.predict(data, dt)

        # update step
        height = np.array(np.log(presval/p0)/(-alpha)).reshape((1, 1))
        heights += [height]
        kf.update(spiral.point_at_z(height).reshape((3, 1)), H_PRESSURE, R_PRESSURE)


    if datatype == 'wifi':
        # reorganize data
        data, ti = data
        position = best_rf.predict(data)[0]

        # predict step
        kf.predict(np.array([0, 0, 0]).reshape((3, 1)), dt)

        # update
        position = np.array(position).flatten()
        input = position.reshape((3, 1))
        pwifi += [position]
        kf.update(input, H_WIFI, R_WIFI)

    pos += [kf.xhat.flatten()[:3]]
    times += [ti]

pos = np.array(pos)
pwifi = np.array(pwifi) - offset
heights = np.array(heights)
print('Done.')

"""
Filter/Process output
"""
# print('Filtering and postprocessing output...', end='')
# from scipy.signal import butter, sosfiltfilt

# sos = butter(200, .3, btype='low', output='sos')
# Xs = sosfiltfilt(sos, pos[:, X])
# Ys = sosfiltfilt(sos, pos[:, Y])
# pos[:, X] = Xs
# pos[:, Y] = Ys
# print('Done.')

"""
Evaluate Results
"""
print('Evaluating Results...')
ateKALMAN, rteKALMAN = compute_ate_rte(np.concatenate((np.array(times).reshape((-1, 1)), pos), axis=1), 
                                    np.concatenate((np.array(gt_timestamp).reshape((-1, 1)), gt_position), axis=1))
print(f'kalman filter ate: {ateKALMAN} rte: {rteKALMAN}')
avgError = compute_average_trajectory_error(np.concatenate((np.array(times).reshape((-1, 1)), pos), axis=1), 
                                    np.concatenate((np.array(gt_timestamp).reshape((-1, 1)), gt_position), axis=1))
print(f'Average Trajectory Error {avgError}')

"""
Plot Results
"""

print('Plotting Results...')
# plot positions as functions of time
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(pos[:, 0], pos[:, 1], pos[:, 2], 'blue')
ax.plot3D(pwifi[:, 0], pwifi[:, 1], pwifi[:, 2], 'green')
# ax.plot3D(np.zeros_like(heights.flatten()) + 2, ts[1:], heights.flatten(), 'red')
ax.plot3D(gt_position[:, 0], gt_position[:, 1], gt_position[:, 2], 'gray')
plt.show()