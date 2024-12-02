"""
Random Forest Regression Based Wifi-Fingerprinting Localization
"""

"""
Configuration
"""
GRIDSEARCH = True
SAVEMODEL = True

"""
Imports
"""
import numpy as np
from hlxon_hdf5io import *
from sklearn.ensemble import RandomForestRegressor
from sklearn.metrics import r2_score
from sklearn.model_selection import  train_test_split
import matplotlib.pyplot as plt


"""
Load Data
"""
dataset = readAll()

"""
Define BSSID mapping
"""
bssids = []
for sequence in dataset:
    _, _, _, _, _, _, wifidata, gt_timestamp, gt_position, _ = sequence

    bssids += [np.array([[row[i].decode() for i in range(2, len(row), 2)] for row in wifidata])]

bssidMap = np.unique(np.concatenate(bssids).flatten())


"""
Preprocess Data and Split Data
"""
Xs = []
ys = []

for sequence in dataset:
    _, _, _, _, _, _, wifidata, gt_timestamp, gt_position, _ = sequence
    ts = np.array([row[0] for row in wifidata])
    cnts = np.array([row[1] for row in wifidata])
    bssids = np.array([[np.argmax(row[i].decode() == bssidMap) for i in range(2, len(row), 2)] for row in wifidata]) # mapped to indices here
    rssis = np.array([[row[i] for i in range(3, len(row), 2)]  for row in wifidata])

    Z = 2
    positions = np.array(gt_position)[:, :]
    gt_ts = np.array(gt_timestamp)

    N = len(wifidata)

    # interpolate over timestamps
    lerped_positions = np.zeros((N, 3))
    for i in range(N):
        if ts[i] in gt_ts:
            lerped_positions[i] = positions[np.argmax(gt_ts == ts[i])]
        else:
            # lerp
            ix_2 = np.argmax(gt_ts > ts[i])
            ix_1 = gt_ts.shape[0] - np.argmax(np.flip(gt_ts, axis=0) < ts[i]) - 1
            

            # lerp formula: y12 = y1 + (t12 - t1) * (y2-y1)/(t2-t1 + stability epsilon)
            lerped_positions[i] = positions[ix_1] + (ts[i]-gt_ts[ix_1])*(positions[ix_2] - positions[ix_1])/(gt_ts[ix_2] - gt_ts[ix_1] + 1e-9)


    # define inputs and outputs
    Xi = np.ones((N, bssidMap.shape[0])) * -100
    rssis = rssis.reshape((N, -1))
    for i in range(N):
        indices = bssids[i]
        for j, k in enumerate(indices):
            Xi[i][k] *= 0
            Xi[i][k] += rssis[i][j]

    yi = lerped_positions

    Xs += [Xi]
    ys += [yi]

# split data
X = np.concatenate(Xs[:-1])
y = np.concatenate(ys[:-1])

X_unseen, y_unseen = Xs[-1], ys[-1]

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.15)


"""
Load Model
"""
from pickle import load
with open(os.path.join("model", "wifi_model.pkl"), "rb") as f:
    best_rf = load(f)

"""
Evaluate Model on test split (Quantitative)
"""
# run on test set
y_pred = best_rf.predict(X_test)

# average distance error
ade = np.mean(np.linalg.norm(y_pred - y_test, axis=1))
print(f'test set distance average distance error: {ade}')

# max distance error
mde = np.linalg.norm(y_pred - y_test, axis=1).max()
print(f'test set distance max distance error: {mde}')

# mean squared error over distances
msde = np.mean(np.linalg.norm(y_pred - y_test, axis=1)**2)
print(f'test set distance mean squared distance error: {msde}')

# r2 score
r2 = r2_score(y_test, y_pred)
print(f'test r2 score: {r2}')

