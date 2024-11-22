import numpy as np
import matplotlib.pyplot as plt
from hlxon_hdf5io import *
from scipy.spatial.transform import Rotation
from filters.HelixonKalmanFilter import *
from metrics import *
import time
from model.spiral_model import *

# get data from hdf5
raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure, wifidata, gt_timestamp, gt_position, gt_orientation = readHDF5('dwifi')

p0 = max(raw_pressure)

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

global_accel_z = accel[:,Z].reshape(N, 1, 1)

# remove gravity vector
# minus since z axis is flipped
# global_accel_z -= 9.81

# --------------------------------
# kalman filter
# --------------------------------
"""
State Vector

format:
    - [0:3] position z (meters global coords)
    - [6:9] velocity z (m/s global coords)
"""


# P (measurement cov mat)
P = np.identity(2) * .01
# Q (process noise)
Q = np.identity(2) * .1
# R (measurement noise)
R = np.identity(1) * 2.
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

def align_spirals(predicted_positions, gt_positions):

    predicted_positions = np.array(predicted_positions)
    gt_positions = np.array(gt_positions)

    # Find x, y bounds for predicted positions
    x_max_pred = np.max(predicted_positions[:, 0])
    x_min_pred = np.min(predicted_positions[:, 0])
    y_max_pred = np.max(predicted_positions[:, 1])
    y_min_pred = np.min(predicted_positions[:, 1])

    # Find x, y bounds for ground truth positions
    x_max_gt = np.max(gt_positions[:, 0])
    x_min_gt = np.min(gt_positions[:, 0])

    y_max_gt = np.max(gt_positions[:, 1])
    y_min_gt = np.min(gt_positions[:, 1])

    # Compute centroids
    center_xy_pred = [(x_max_pred + x_min_pred) / 2, (y_max_pred + y_min_pred) / 2]
    center_xy_gt = [(x_max_gt + x_min_gt) / 2, (y_max_gt + y_min_gt) / 2]

    print(center_xy_pred, center_xy_gt)

    predicted_positions[:,0] += (center_xy_gt[0] - center_xy_pred[0])

    predicted_positions[:,1] += (center_xy_gt[1] - center_xy_pred[1])

    return predicted_positions




kf = HelixonKalmanFilter(getA, getB, P, Q, R, H)

# all ys (measurements) for kalman filter
heights = np.log(pres/p0)/(-alpha)
ys = heights

# all us (control inputs) for kalman filter
us = global_accel_z

# creating spiral model
spiral_pitch = 5 #m
spiral_radius = 8 #m
path_width = 2.4 #m
Spiral = Spiral(spiral_pitch, spiral_radius, path_width)


TARGET = 'offline_spiral'
# PLOTTING
if TARGET == 'offline':

    # Run Kalman Filter offline
    predicted_heights = kf.run_offline(us, ys, ts)[:, 0].reshape(-1, 1)
    predicted_positions = []
    for height in predicted_heights:
        predicted_positions.append(Spiral.point_at_z(height))

    # ATE and RTE for heights only
    ateKALMAN, rteKALMAN = compute_ate_rte(np.concatenate((np.array(ts).reshape((-1, 1)), predicted_heights*np.array([0, 0, 1])), axis=1), 
                                        np.concatenate((np.array(gt_timestamp).reshape((-1, 1)), gt_position*np.array([0, 0, 1])), axis=1))

    ateH, rteH = compute_ate_rte(np.concatenate((np.array(ts).reshape((-1, 1)), heights*np.array([0, 0, 1])), axis=1), 
                            np.concatenate((np.array(gt_timestamp).reshape((-1, 1)), gt_position*np.array([0, 0, 1])), axis=1))

    print(f'kalman filter ate: {ateKALMAN} rte: {rteKALMAN}\nheights ate: {ateH} rte: {rteH}')

    # plot heights as functions of time
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(np.zeros_like(predicted_heights[:, 0]) + 1, ts, predicted_heights[:, 0], 'blue')
    ax.plot3D(np.zeros_like(heights.flatten()) + 2, ts, heights.flatten(), 'red')
    ax.plot3D(np.zeros_like(gt_position[:, 0]), gt_timestamp, gt_position[:, 2], 'gray')
    ax.plot3D(np.zeros_like(predicted_positions[:,0]), gt_timestamp, gt_position[:, 2], 'green')
    plt.show()


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

elif TARGET == 'offline_spiral':

    # Run Kalman Filter offline
    predicted_heights = kf.run_offline(us, ys, ts)[:, 0].reshape(-1, 1)
    predicted_positions = []
    for height in predicted_heights:
        predicted_positions.append(Spiral.point_at_z(height))

    predicted_positions = align_spirals(predicted_positions, gt_position)

    # Predicted positions (Spiral model)
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    predicted_positions = np.array(predicted_positions)  # Convert list to NumPy array
    ax.plot3D(predicted_positions[:, 0], predicted_positions[:, 1], predicted_positions[:, 2], 'red', label='Predicted Positions (Kalman)')
    ax.plot3D(gt_position[:, 0], gt_position[:, 1], gt_position[:, 2], 'blue', label='Groundtruth Position')

    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()