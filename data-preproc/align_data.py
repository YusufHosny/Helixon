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
from scipy import signal

# creating spiral model
spiral_pitch = 4.2 #m
spiral_radius = 8 #m
path_width = 2.4 #m
spiral = Spiral(spiral_pitch, spiral_radius, path_width)


# convenience
X, Y, Z = 0, 1, 2
def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])

def angular_distance(b, a):
    return np.where(np.abs(b-a) < 180, b-a, np.where((b-a) > 0, 360 - (b-a), 360 + (b-a)))

def determine_delay(raw_rpy, raw_timestamp, gt_orientation, gt_timestamp):
    ts = np.array(raw_timestamp)

    rots = np.array([Rotation.from_quat(q).as_euler('xyz', degrees=True) for q in gt_orientation]) # change quat gt rots to euler
    rots -= rots[0] # normalize to initial rot
    rots = rots[:, ::-1] # reverse order of xyz to match ndof
    rots[:, X] *= -1 # invert X rotations to match ndof
    rots = np.array([Rotation.from_euler('xyz', r, degrees=True).as_euler('xyz', degrees=True) for r in rots])

    # lerp rots
    N = len(rots)

    # interpolate over timestamps
    lerped_rotations = np.zeros((N, 3))
    for i in range(N):
        if gt_timestamp[i] in ts:
            lerped_rotations[i] = raw_rpy[np.argmax(ts == gt_timestamp[i])]
        else:
            # lerp
            ix_2 = np.argmax(ts > gt_timestamp[i])
            ix_1 = ts.shape[0] - np.argmax(np.flip(ts, axis=0) < gt_timestamp[i]) - 1

            lerped_rotations[i] = raw_rpy[ix_1] + (gt_timestamp[i]-ts[ix_1])*angular_distance(raw_rpy[ix_2], raw_rpy[ix_1])/(ts[ix_2] - ts[ix_1] + 1e-10)

    rpy = lerped_rotations

    # interpolate over time to create single sampling rate
    fs = len(gt_timestamp) / gt_timestamp[-1]
    CNT = int(fs*gt_timestamp[-1])
    unified_ts = np.linspace(0, gt_timestamp[-1], CNT)

    lerpedrpy = np.zeros((CNT, 3))
    lerpedrots = np.zeros((CNT, 3))

    for i in range(CNT):
        if unified_ts[i] in gt_timestamp:
            lerpedrpy[i] = rpy[np.argmax(gt_timestamp == unified_ts[i])]
            lerpedrots[i] = rots[np.argmax(gt_timestamp == unified_ts[i])]

        else:
            # lerp
            ix_2 = np.argmax(gt_timestamp > unified_ts[i])
            ix_1 = gt_timestamp.shape[0] - np.argmax(np.flip(gt_timestamp, axis=0) < unified_ts[i]) - 1

            lerpedrpy[i] = rpy[ix_1] + (unified_ts[i]-gt_timestamp[ix_1])*angular_distance(rpy[ix_2], rpy[ix_1])/(gt_timestamp[ix_2] - gt_timestamp[ix_1] + 1e-10)
            lerpedrots[i] = rots[ix_1] + (unified_ts[i]-gt_timestamp[ix_1])*angular_distance(rots[ix_2], rots[ix_1])/(gt_timestamp[ix_2] - gt_timestamp[ix_1] + 1e-10)

    rots = lerpedrots
    rpy = lerpedrpy


    fs = len(unified_ts) / unified_ts[-1]
    n = len(rpy[:, X])

    corr = signal.correlate(rots[:, X], rpy[:, X], mode='same') / np.sqrt(signal.correlate(rpy[:, X], rpy[:, X], mode='same')[int(n/2)] * signal.correlate(rots[:, X], rots[:, X], mode='same')[int(n/2)])

    delay_arr = np.linspace(-0.5*n, 0.5*n, n)
    delay = delay_arr[np.argmax(corr)]

    return delay / fs


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

    # normalize gt and raw rotations to (0, 0, 0)
    raw_rpy -= raw_rpy[0]
    raw_rpy = np.array([Rotation.from_euler('xyz', r, degrees=True).as_euler('xyz', degrees=True) for r in raw_rpy]) # clamp raw rotations to correct range 
    gt_orientation = np.array(gt_orientation)
    r0 = Rotation.from_quat(gt_orientation[0]).inv().as_quat()
    gt_orientation = list(map(lambda q2: quaternion_multiply(r0, q2), gt_orientation))

    # remove pressure outliers (sensor not ready yet)
    p0 = raw_pressure.max()
    raw_pressure[np.where(raw_pressure < 100000)] = p0

    # apply cross correlation
    delay = determine_delay(raw_rpy, raw_timestamp, gt_orientation, gt_timestamp)
    gt_timestamp = np.array(gt_timestamp)
    gt_orientation = np.array(gt_orientation)
    gt_timestamp -= delay
    gt_orientation = gt_orientation[np.where(gt_timestamp >= 0)]
    gt_position = gt_position[np.where(gt_timestamp >= 0)]
    gt_timestamp = gt_timestamp[np.where(gt_timestamp >= 0)]

    raw_data = np.array([np.stack((t, ndof[0], ndof[1], ndof[2], ndof[3], ndof[4], ndof[5], ndof[6], ndof[7], ndof[8], rpy[0], rpy[1], rpy[2], bno, bmp, pres)) for (t, ndof, rpy, bno, bmp, pres) in zip(raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure)])
    gt_data = np.array([np.stack((t, pi[0], pi[1], pi[2], rpyi[0], rpyi[1], rpyi[2], rpyi[3])) for (t, pi, rpyi) in zip(gt_timestamp, posi, gt_orientation)])
    storeAsHDF5_path(file_path, raw_data, gt_data, wifidata)

PLOT = False
if PLOT:
    # plot all datasets
    X, Y, Z = 0, 1, 2
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    gt_pos = []
    for pi, ci in zip(pos, ['blue', 'red', 'green', 'yellow', 'gray', 'black', 'orange', 'purple']):
        ax.plot3D(pi[:, X], pi[:, Y], pi[:, Z], ci)
    plt.show()
