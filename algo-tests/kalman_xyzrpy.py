import numpy as np
import matplotlib.pyplot as plt
from hlxon_hdf5io import *
from scipy.spatial.transform import Rotation
from filters.HelixonKalmanFilter import *
from metrics import *
import time

# get data from hdf5
raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure, wifidata, gt_timestamp, gt_position, gt_orientation = readHDF5('spiral2')

# p0 is first real pressure measurement
p0 = raw_pressure[np.argmax(np.array(raw_pressure) > 1e5)]

# convenience
X, Y, Z = 0, 1, 2
N = len(raw_timestamp)

# remove offsets gt pos and orientation
gt_position = np.array(gt_position)
gt_position -= gt_position[0]
gt_orientation = np.array(gt_orientation)
gt_orientation -= gt_orientation[0]

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

# remove gravity vector
# minus since z axis is flipped
# accel[:, Z] -= 9.81


# --------------------------------
# kalman filter
# --------------------------------
"""
State Vector

format:
    - [0:3] positions x y z (meters global coords)
    - [3:6] orientations roll pitch yaw (degrees)
    - [6:9] velocities x y z (m/s global coords)
"""


# P (measurement cov mat)
P = np.identity(9) * 0.5
# Q (process noise)
Q = np.identity(9) * 0.5
# R (measurement noise)
R = np.identity(1) * .01
# H (measurement matrix)
H = np.array([
    [0., 0., 1., 0., 0., 0., 0., 0., 0.], 
])

# function to get A (state transition matrix) for certain dt
def getA(dt: float):
    return np.array([
# positions |  |  orients  |  |   velos   |   
[1., 0., 0.,    0., 0., 0.,    dt, 0., 0. ],  # px
[0., 1., 0.,    0., 0., 0.,    0., dt, 0. ],  # py 
[0., 0., 1.,    0., 0., 0.,    0., 0., dt ],  # pz 
[0., 0., 0.,    1., 0., 0.,    0., 0., 0. ],  # r
[0., 0., 0.,    0., 1., 0.,    0., 0., 0. ],  # p
[0., 0., 0.,    0., 0., 1.,    0., 0., 0. ],  # y
[0., 0., 0.,    0., 0., 0.,    1., 0., 0. ],  # vx
[0., 0., 0.,    0., 0., 0.,    0., 1., 0. ],  # vy
[0., 0., 0.,    0., 0., 0.,    0., 0., 1. ],  # vz
    ])

# function to get B (control transition matrix) for certain dt
def getB(dt: float):
    return np.array([
#               accels              |   angvels   | 
[.5*dt**2,  0.,         0.,          0., 0., 0. ],  # px
[0.,        .5*dt**2,   0.,          0., 0., 0. ],  # py 
[0.,        0.,         .5*dt**2,    0., 0., 0. ],  # pz 
[0.,        0.,         0.,          dt, 0., 0. ],  # r
[0.,        0.,         0.,          0., dt, 0. ],  # p
[0.,        0.,         0.,          0., 0., dt ],  # y
[dt,        0.,         0.,          0., 0., 0. ],  # vx
[0.,        dt,         0.,          0., 0., 0. ],  # vy
[0.,        0.,         dt,          0., 0., 0. ],  # vz
    ])


kf = HelixonKalmanFilter(getA, getB, P, Q, R, H)

# all ys (measurements) for kalman filter
heights = np.log(pres/p0)/(-alpha)
ys = heights

# all us (control inputs) for kalman filter
us = np.concatenate((accel, gyro), axis=1).reshape((-1, 6, 1))

# PLOTTING
# TARGET = 'height'
# TARGET = 'all'
TARGET = 'real_time'

if TARGET == 'height':

    pos = kf.run_offlne(us, ys, ts)[:, :3]
    # ATE and RTE for heights only
    ateKALMAN, rteKALMAN = compute_ate_rte(np.concatenate((np.array(ts).reshape((-1, 1)), pos*np.array([0, 0, 1])), axis=1), 
                                        np.concatenate((np.array(gt_timestamp).reshape((-1, 1)), gt_position*np.array([0, 0, 1])), axis=1))

    ateH, rteH = compute_ate_rte(np.concatenate((np.array(ts).reshape((-1, 1)), heights*np.array([0, 0, 1])), axis=1), 
                            np.concatenate((np.array(gt_timestamp).reshape((-1, 1)), gt_position*np.array([0, 0, 1])), axis=1))

    print(f'kalman filter ate: {ateKALMAN} rte: {rteKALMAN}\nheights ate: {ateH} rte: {rteH}')

    # plot heights as functions of time
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(np.zeros_like(pos[:, 0]) + 1, ts, pos[:, 2], 'blue')
    ax.plot3D(np.zeros_like(heights.flatten()) + 2, ts, heights.flatten(), 'red')
    ax.plot3D(np.zeros_like(gt_position[:, 0]), gt_timestamp, gt_position[:, 2], 'gray')
    plt.show()

elif TARGET == 'all':

    pos = kf.run_offlne(us, ys, ts)[:, :3]
    ateKALMAN, rteKALMAN = compute_ate_rte(np.concatenate((np.array(ts).reshape((-1, 1)), pos), axis=1), 
                                        np.concatenate((np.array(gt_timestamp).reshape((-1, 1)), gt_position), axis=1))
    print(f'kalman filter ate: {ateKALMAN} rte: {rteKALMAN}')

    # plot positions as functions of time
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(pos[:, 0], pos[:, 1], pos[:, 2], 'blue')
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
