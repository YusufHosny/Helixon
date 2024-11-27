"""
Random Forest Based Wifi-Fingerprinting Localization
"""


"""
Imports
"""
import pandas as pd
import numpy as np
from hlxon_hdf5io import *
from itertools import count
from sklearn.ensemble import RandomForestRegressor
from sklearn.metrics import accuracy_score, confusion_matrix, precision_score, recall_score, ConfusionMatrixDisplay
from sklearn.model_selection import GridSearchCV, train_test_split
from scipy.stats import randint
from sklearn.tree import export_graphviz
from IPython.display import Image
import graphviz


"""
Load and Format Data
"""
raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure, wifidata, gt_timestamp, gt_position, gt_orientation = readHDF5('synthetic')

ts = np.array([row[0] for row in wifidata])
cnts = np.array([row[1] for row in wifidata])
bssids = np.array([[row[i].decode() for i in range(2, len(row), 2)] for row in wifidata])
rssis = np.array([[row[i] for i in range(3, len(row), 2)]  for row in wifidata])

Z = 2
heights = np.array(gt_position)[:, Z]

gt_ts = np.array(gt_timestamp)

N = len(wifidata)

"""
Preprocess Data and Split Data
"""
# define unique bssid set and map bssids into this dict
bssidMap, mappedBssids = np.unique(bssids, return_inverse=True)

# interpolate over timestamps
lerped_heights = np.zeros((N,))
for i in range(N):
    if ts[i] in gt_ts:
        lerped_heights[i] = gt_ts[np.argmax(gt_ts == ts[i])]
    else:
        # lerp
        ix_2 = np.argmax(gt_ts > ts[i])
        ix_1 = gt_ts.shape[0] - np.argmax(np.flip(gt_ts, axis=0) < ts[i])
        

        # lerp formula: y12 = y1 + (t12 - t1) * (y2-y1)/(t2-t1 + stability epsilon)
        lerped_heights[i] = heights[ix_1] + (ts[i]-gt_ts[ix_1])*(heights[ix_2] - heights[ix_1])/(gt_ts[ix_2] - gt_ts[ix_1] + 1e-9)


# define inputs and outputs
X = np.concatenate((mappedBssids.reshape((N, -1)), rssis), axis=1)
y = lerped_heights

# split data
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2)


"""
Tune and Train Model
"""
# create model
rf = RandomForestRegressor()

# hyperparamter grid search
param_dist = {'n_estimators': np.floor(np.linspace(50, 500, 5)).astype(np.int16),
              'max_depth': np.linspace(1, 20, 5, dtype=np.int8)}

# grid search over params
grid_search = GridSearchCV(rf, 
    param_grid = param_dist,
    verbose=3)

# train model
grid_search.fit(X_train, y_train)


# store best hyperparams and model
best_rf = grid_search.best_estimator_
print(f'best hyperparams: {grid_search.best_params_}')

"""
Evaluate Model
"""
# run on test set
y_pred = best_rf.predict(X_test)

# accuracy
mse = np.mean((y_pred - y_test)**2)
print(f'test set MSE: {mse}')

# random prediction
ix = np.random.randint(0, len(X_test))
print(f'inp: {X_test[ix]}, predicted: {y_pred[ix]}, truth: {y_test[ix]}')

"""
Save Model
"""