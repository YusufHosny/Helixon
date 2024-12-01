"""
Aligns all data in the data/ folder
Writes the output data to aligned_data/
"""

import matplotlib.pyplot as plt
from hlxon_hdf5io.hlxon_hdf5io import *
import os
import numpy as np
from scipy.spatial.transform import Rotation
from model.spiral_model import Spiral


# creating spiral model
spiral_pitch = 4.2 #m
spiral_radius = 8 #m
path_width = 2.4 #m
spiral = Spiral(spiral_pitch, spiral_radius, path_width)


# convenience
X, Y, Z = 0, 1, 2

data_raw = readAll()
pos = []
for di in data_raw:
    _, _, _, _, _, _, _, _, gt_position, _ = di

    gt_pos = np.array(gt_position)
    gt_pos[:, Z] -= gt_pos[0, Z]
    
    spiral.center_spiral(gt_pos)
    pos += [gt_pos]


# align alot of xy vectors
for i in range(1, len(pos)):
    rot, _ = Rotation.align_vectors([np.mean(pos[0][:j], axis=0)*np.array([1., 1., 0.]) for j in range(10, 400, 10)], 
                                    [np.mean(pos[i][:j], axis=0)*np.array([1., 1., 0.]) for j in range(10, 400, 10)])
    pos[i] = rot.apply(pos[i])


# Load dataset names
dataset_names = os.listdir(os.path.join('./', 'data'))

# Creating a new .h5 file in the HDF5s folder
for name, di, posi in zip(dataset_names, data_raw, pos):
    raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure, wifidata, gt_timestamp, gt_position, gt_orientation = di
    file_path = os.path.join("aligned_data", f"{name}")
    os.makedirs("aligned_data", exist_ok=True)  # Ensure the directory exists

    raw_data = np.array([np.stack((t, ndof[0], ndof[1], ndof[2], ndof[3], ndof[4], ndof[5], ndof[6], ndof[7], ndof[8], rpy[0], rpy[1], rpy[2], bno, bmp, pres)) for (t, ndof, rpy, bno, bmp, pres) in zip(raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure)])
    gt_data = np.array([np.stack((t, pi[0], pi[1], pi[2], rpyi[0], rpyi[1], rpyi[2], rpyi[3])) for (t, pi, rpyi) in zip(gt_timestamp, posi, gt_orientation)])
    storeAsHDF5_path(file_path, raw_data, gt_data, wifidata)

# plot all datasets
X, Y, Z = 0, 1, 2
fig = plt.figure()
ax = plt.axes(projection='3d')
gt_pos = []
for pi, ci in zip(pos, ['blue', 'red', 'green', 'yellow', 'gray', 'black', 'orange', 'purple']):
    ax.plot3D(pi[:, X], pi[:, Y], pi[:, Z], ci)
plt.show()
