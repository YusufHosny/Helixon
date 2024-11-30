import matplotlib.pyplot as plt
from hlxon_hdf5io.hlxon_hdf5io import *
import os
import numpy as np
from scipy.spatial.transform import Rotation
import torch


dataset_names = os.listdir(os.path.join('./', 'data'))

pos = []
for name in dataset_names:
    raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure, wifidata, gt_timestamp, gt_position, gt_orientation = readHDF5(name.replace('.h5', ''))
    gt_position -= gt_position[0]       

    pos += [np.array(gt_position)]

for i in range(1, len(pos)):
    #rot, _ = Rotation.align_vectors(np.mean(pos[0][:], axis=0)*np.array([1., 1., 0.]), np.mean(pos[i][:140], axis=0)*np.array([1., 1., 0.]))
    rot, _ = Rotation.align_vectors([np.mean(pos[0][:j], axis=0)*np.array([1., 1., 0.]) for j in range(10, 400, 10)], 
                                    [np.mean(pos[i][:j], axis=0)*np.array([1., 1., 0.]) for j in range(10, 400, 10)])
    pos[i] = rot.apply(pos[i])

# plot all datasets
X, Y, Z = 0, 1, 2
fig = plt.figure()
ax = plt.axes(projection='3d')
for pi, ci in zip(pos, ['blue', 'red', 'green', 'yellow', 'gray', 'black', 'orange', 'purple']):
    ax.plot3D(pi[:, X], pi[:, Y], pi[:, Z], ci)
plt.show()
