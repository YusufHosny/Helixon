import numpy as np
import matplotlib.pyplot as plt
from hlxon_hdf5io import *
from scipy.spatial.transform import Rotation
from filters.HelixonKalmanFilter import *
from metrics import *
import time

# get data from hdf5
raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure, wifidata, gt_timestamp, gt_position, gt_orientation = readHDF5('RandomUDP6')

# p0 is first real pressure measurement
p0 = np.mean(raw_pressure[:15])

# convenience
X, Y, Z = 0, 1, 2
N = len(raw_timestamp)

# get sensor data
araw = np.array(raw_9dof[:, :3])
gyro = np.array(raw_9dof[:, 3:6])
magn = np.array(raw_9dof[:, 6:])
pres = np.array(raw_pressure).reshape((-1, 1))
alpha = 1.16e-4
ts = np.array(raw_timestamp)*1e-6
gt_timestamp = np.array(gt_timestamp)*1e-6

# rotate acceleration to global coords
accel = np.zeros_like(araw)
for i, orientation in enumerate(raw_rpy):
    
    # check if quat or euler
    if orientation.shape[-1] == 4:
        rot = Rotation.from_quat(orientation).inv()
    else:
        rot = Rotation.from_euler('xyz', orientation, degrees=True).inv()
    accel[i] = rot.apply(araw[i])


# --------------------------------
# kalman filter
# --------------------------------
"""
State Vector

format:
    - [0:3] positions x y z (meters global coords)
    - [0:6] velocities x y z (m/s global coords)
"""


# P (measurement cov mat)
P = np.identity(6) * 0.5
# Q (process noise)
Q = np.identity(6) * 0.5
# R (measurement noise)
R_PRESSURE = np.diagonal([0., 0., 1., 0., 0., 0.]) * .01
R_WIFI = np.diagonal([1., 1., 1., 0., 0., 0.]) * 2.
# H (measurement matrix)
H_PRESSURE = np.array([
    [0., 0., 1., 0., 0., 0.], 
])
H_WIFI = np.array([
    [1., 1., 1., 0., 0., 0.], 
])

# function to get A (state transition matrix) for certain dt
def getA(dt: float):
    return np.array([
# positions |  |  orients  |  |   velos   |   
[1., 0., 0.,    0., 0., 0.,    dt, 0., 0. ],  # px
[0., 1., 0.,    0., 0., 0.,    0., dt, 0. ],  # py 
[0., 0., 1.,    0., 0., 0.,    0., 0., dt ],  # pz
[0., 0., 0.,    0., 0., 0.,    1., 0., 0. ],  # vx
[0., 0., 0.,    0., 0., 0.,    0., 1., 0. ],  # vy
[0., 0., 0.,    0., 0., 0.,    0., 0., 1. ],  # vz
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

# all ys (measurements) for kalman filter
heights = np.log(pres/p0)/(-alpha)
ys = heights

# all us (control inputs) for kalman filter
us = np.concatenate((accel), axis=1).reshape((-1, 3, 1))

# PLOTTING
TARGET = 'all'

if TARGET == 'all':

    pos = kf.run_offline(us, ys, ts)[:, :3]
    print(gt_position)
    print("Sizes: ",np.max(gt_position[:, 0]) - np.min(gt_position[:, 0]), np.max(gt_position[:, 1])- np.min(gt_position[:, 1]), np.max(gt_position[:, 2])- np.min(gt_position[:, 2]))
    ateKALMAN, rteKALMAN = compute_ate_rte(np.concatenate((np.array(ts).reshape((-1, 1)), pos), axis=1), 
                                        np.concatenate((np.array(gt_timestamp).reshape((-1, 1)), gt_position), axis=1))
    print(f'kalman filter ate: {ateKALMAN} rte: {rteKALMAN}')

    # plot positions as functions of time
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    # ax.plot3D(pos[:, 0], pos[:, 1], pos[:, 2], 'blue')
    ax.plot3D(gt_position[:, 0], gt_position[:, 1], gt_position[:, 2], 'gray')
    plt.show()

elif TARGET == 'real_time':
    tot_time = 0

    for i in range(1, N):
        dt = ts[i] - ts[i - 1]
        u = us[i]
        y = ys[i]
        
        t1 = time.perf_counter()
        # position estimate
        estimated_position = kf.run_step(u, y, dt)[:3]

        t2 = time.perf_counter()

        tot_time += t2-t1

        #print(f"Time {ts[i]:.2f}s | Estimated Position: {estimated_position}")

    print("Operating frequency: ", (N-1)/tot_time, " Hz")
