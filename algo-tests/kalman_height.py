from os import path as osp
import numpy as np
import matplotlib.pyplot as plt
from hlxon_hdf5io import *
from scipy.spatial.transform import Rotation
from typing import Callable, Self
from metrics import *
import time

# get data from hdf5
raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure, gt_timestamp, gt_position, gt_orientation = readHDF5('spiral2')

p0 = raw_pressure[np.argmax(np.array(raw_pressure) > 1e5)]
raw_pressure = [p if p > 1e5 else p0 for p in raw_pressure]

# convenience
Z = 2
N = len(raw_timestamp)

# remove offsets gt pos and orientation
gt_position = np.array(gt_position)
gt_position -= gt_position[0]

# get sensor data
araw = np.array(raw_9dof[:, :3])
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

global_accel_z = accel[:,Z]

# remove gravity vector
# minus since z axis is flipped
global_accel_z -= 9.81

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
class HelixonKalmanFilter:

    def __init__(self: Self, getA: Callable[[float], np.ndarray], getB: Callable[[float], np.ndarray], P: np.ndarray, Q: np.ndarray, R: np.ndarray, H: np.ndarray):
        self.getA = getA
        self.getB = getB
        self.P = P
        self.Q = Q
        self.R = R
        self.H = H
        self.xhat = np.zeros((2, 1))

    def predict(self: Self, input: float, dt: float):
        self.A = self.getA(dt)
        self.B = self.getB(dt)

        input = np.array([[input]])  # Convert scalar to 2D array

        
        self.xhat = self.A @ self.xhat + self.B @ input
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self: Self, y: np.ndarray):
        self.K = self.P @ self.H.T @ np.linalg.inv((self.H @ self.P @ self.H.T) + self.R)
        
        self.xhat += self.K @ (y - (self.H @ self.xhat))
        self.P = (np.identity(2) - (self.K @ self.H)) @ self.P

    def run_offlne(self: Self, us: np.ndarray, ys: np.ndarray) -> np.ndarray:
        pos = np.zeros((N, 2))
        for i in range(1, N):
            dt = (ts[i]-ts[i-1])
            kf.predict(us[i], dt)
            kf.update(ys[i])
            pos[i] = kf.xhat[:2].reshape((2,))

        return pos[:,0] # Only interested in height, not velocity
    
    def run_step(self: Self, u: np.ndarray, y: np.ndarray, dt: float) -> np.ndarray:
        self.predict(u, dt)
        self.update(y)
        pos = self.xhat[:2].flatten()
        return pos[0] # Only interested in height, not velocity



# P (measurement cov mat)
P = np.identity(2) * 0.5
# Q (process noise)
Q = np.identity(2) * 0.5
# R (measurement noise)
R = np.identity(1) * .01
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


kf = HelixonKalmanFilter(getA, getB, P, Q, R, H)

# all ys (measurements) for kalman filter
heights = np.log(pres/p0)/(-alpha)
ys = heights

# all us (control inputs) for kalman filter
us = global_accel_z

TARGET = 'offline'
# PLOTTING
if TARGET == 'offline':

    # Run Kalman Filter offline
    predicted_heights = kf.run_offlne(us, ys)

    print(predicted_heights)


elif TARGET == 'online':

    tot_time = 0

    for i in range(1, len(ts)):
        dt = ts[i] - ts[i - 1]
        u = us[i]
        y = ys[i]

        t1 = time.perf_counter()
        # position estimate
        estimated_position = kf.run_step(u, y, dt)

        t2 = time.perf_counter()

        tot_time += t2-t1

        print(f"Time {ts[i]:.2f}s | Estimated Height: {estimated_position}")

    print("Operating frequency: ", (len(ts)-1)/tot_time, " Hz")