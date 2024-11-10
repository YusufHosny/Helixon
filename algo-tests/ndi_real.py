from hlxon_hdf5io import *
from os import path as osp
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

# get data from hdf5
raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure, gt_timestamp, gt_position, gt_orientation = readHDF5('up_and_down')

# get sensor data
accel = np.array(raw_9dof[:, :3])
gyro = np.array(raw_9dof[:, 3:6])
magn = np.array(raw_9dof[:, 6:])

roll = raw_rpy[:, 0]
pitch = raw_rpy[:, 1]
yaw = raw_rpy[:, 2]


# remove gravity vector
for i, (ai, ri, pi, yi) in enumerate(zip(accel, roll, pitch, yaw)):
    rot = Rotation.from_euler('xyz', [ri, pi, yi], degrees=True)
    ai -= rot.apply([0, 0, 9.81])


# dead reckoning ndi on real data
pos = np.zeros((120, 3)) + np.array(gt_position[0, :]).reshape(1, 3)
vi = np.array([0., 0., 0.])
tprev = 0
for i, (ti, psi, ai, gi, mi, ri, pi, yi) in enumerate(zip(raw_timestamp, pos, accel, gyro, magn, roll, pitch, yaw)):
    rot = Rotation.from_euler('xyz', [-ri, -pi, -yi], degrees=True)
    vi +=  rot.apply(ai)*(ti-tprev)*1e-6
    psi += [vij*(ti-tprev)*1e-6 for vij in vi]
    tprev = ti

# plot ground truth
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(gt_position[:120, 0], gt_position[:120, 1], gt_position[:120, 2], 'gray')

# plot ndi
ax.plot3D(pos[:, 0], pos[:, 1], pos[:, 2], 'blue')

plt.show()