import numpy as np
import matplotlib.pyplot as plt
from hlxon_hdf5io import *
from scipy.spatial.transform import Rotation
from filters.HelixonKalmanFilter import *
from metrics import *
import time
from model.spiral_model import *

dataset = readAll()

for sequence in dataset:
    # get data from hdf5
    raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure, wifidata, gt_timestamp, gt_position, gt_orientation = sequence

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
