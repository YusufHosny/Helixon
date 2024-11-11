from os import path as osp
import numpy as np
import matplotlib.pyplot as plt
from hlxon_hdf5io import *
from scipy.spatial.transform import Rotation
from typing import Callable, Self

# get data from hdf5
raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure, gt_timestamp, gt_position, gt_orientation = readHDF5('synthetic')

# convenience
X, Y, Z = 0, 1, 2
N = len(raw_timestamp)

# get sensor data
araw = np.array(raw_9dof[:, :3])
gyro = np.array(raw_9dof[:, 3:6])
magn = np.array(raw_9dof[:, 6:])
pres = np.array(raw_pressure).reshape((-1, 1))
p0 = 101325 # Pa
alpha = 1.16e-4
ts = raw_timestamp

# rotate acceleration to global coords
accel = np.zeros_like(araw)
for i, rpyi in enumerate(gt_orientation):
    rot = Rotation.from_euler('xyz', rpyi, degrees=True).inv()
    accel[i] = rot.apply(araw[i])

# remove gravity vector
accel[:, Z] += 9.81

# --------------------------------
# kalman filter
# --------------------------------
"""
State Vector

format:
    - [0:3] positions x y z (meters global coords)
    - [3:6] orientations roll pitch yaw (degrees)
    - [6:9] accelerations x y z (m/s^2 global coords)
    - [9:12] velocities x y z (m/s global coords)
    - [12:15] angular velocities roll pitch yaw (rad/s global coords)
"""
class HelixonKalmanFilter:

    def __init__(self: Self, getA: Callable[[float], np.ndarray], getB: Callable[[float], np.ndarray], P: np.ndarray, Q: np.ndarray, R: np.ndarray, H: np.ndarray):
        self.getA = getA
        self.getB = getB
        self.P = P
        self.Q = Q
        self.R = R
        self.H = H
        self.xhat = np.zeros((9, 1))

    def predict(self: Self, input: np.ndarray ,dt: float):
        self.A = self.getA(dt)
        self.B = self.getB(dt)
        
        self.xhat = self.A @ self.xhat + self.B @ input
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self: Self, y: np.ndarray):
        self.K = self.P @ self.H.T @ np.linalg.inv((self.H @ self.P @ self.H.T) + self.R)
        
        self.xhat += self.K @ (y - (self.H @ self.xhat))
        self.P = (np.identity(9) - (self.K @ self.H)) @ self.P

    def run_offlne(self: Self, us: np.ndarray, ys: np.ndarray) -> np.ndarray:
        pos = np.zeros((N, 3))
        for i in range(1, N):
            dt = ts[i]-ts[i-1]
            kf.predict(us[i], dt)
            kf.update(ys[i])
            pos[i] = kf.xhat[:3].reshape((3,))
        return pos



# P (measurement cov mat)
P = np.identity(9) * 0.0005
# Q (process noise)
Q = np.identity(9) * 0.0005
# R (measurement noise)
R = np.identity(1) * .0
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

pos = kf.run_offlne(us, ys)

# plot synth ndi and gt
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(pos[:, 0], pos[:, 1], pos[:, 2], 'blue')
ax.plot3D(gt_position[:, 0], gt_position[:, 1], gt_position[:, 2], 'gray')
plt.show()
