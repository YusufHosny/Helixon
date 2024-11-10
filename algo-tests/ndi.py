from os import path as osp
import numpy as np
import matplotlib.pyplot as plt
from hlxon_hdf5io import *
from scipy.spatial.transform import Rotation

from metrics import *

# get data from hdf5
raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure, gt_timestamp, gt_position, gt_orientation = readHDF5('synthetic')

# convenience
X, Y, Z = 0, 1, 2
N = len(raw_timestamp)

# get sensor data
araw = np.array(raw_9dof[:, :3])
gyro = np.array(raw_9dof[:, 3:6])
magn = np.array(raw_9dof[:, 6:])
ts = raw_timestamp

# rotate acceleration to global coords
accel = np.zeros_like(araw)
for i, rpyi in enumerate(gt_orientation):
    rot = Rotation.from_euler('xyz', rpyi, degrees=True).inv()
    accel[i] = rot.apply(araw[i])

# remove gravity vector
accel[:, Z] += 9.81

# dead reckoning ndi on synth
pos = np.zeros((N, 3))
vi = np.array([0., 0., 0.])
for i in range(1, N):
    dt = (ts[i]-ts[i-1])
    vi += accel[i]*dt
    pos[i] = pos[i-1] + vi*dt + 0.5*accel[i]*dt**2

# get error and print    
ate, rte = compute_ate_rte(pos, gt_position, 10)
print(f'Absolute Trajectory Error: {ate}\nRelative Trajectory Error: {rte}')

# plot synth ndi and gt
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(pos[:, 0], pos[:, 1], pos[:, 2], 'blue')
ax.plot3D(gt_position[:, 0], gt_position[:, 1], gt_position[:, 2], 'gray')
plt.show()
