"""
Random Forest Regression Based Wifi-Fingerprinting Localization
"""


"""
Imports
"""
import numpy as np
from hlxon_hdf5io import *
from sklearn.ensemble import RandomForestRegressor
from sklearn.metrics import r2_score
from sklearn.model_selection import GridSearchCV, train_test_split


"""
Load Data
"""
# raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure, wifidata, gt_timestamp, gt_position, gt_orientation = readHDF5('NormalUDP1')
dataset = [readAll()[0]]


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
    heights = np.array(gt_position)[:, :]
    gt_ts = np.array(gt_timestamp)

    N = len(wifidata)

    # interpolate over timestamps
    lerped_heights = np.zeros((N, 3))
    for i in range(N):
        if ts[i] in gt_ts:
            lerped_heights[i] = heights[np.argmax(gt_ts == ts[i])]
        else:
            # lerp
            ix_2 = np.argmax(gt_ts > ts[i])
            ix_1 = gt_ts.shape[0] - np.argmax(np.flip(gt_ts, axis=0) < ts[i]) - 1
            

            # lerp formula: y12 = y1 + (t12 - t1) * (y2-y1)/(t2-t1 + stability epsilon)
            lerped_heights[i] = heights[ix_1] + (ts[i]-gt_ts[ix_1])*(heights[ix_2] - heights[ix_1])/(gt_ts[ix_2] - gt_ts[ix_1] + 1e-9)


    # define inputs and outputs
    Xi = np.ones((N, bssidMap.shape[0])) * -100
    rssis = rssis.reshape((N, -1))
    for i in range(N):
        indices = bssids[i]
        for j, k in enumerate(indices):
            Xi[i][k] *= 0
            Xi[i][k] += rssis[i][j]

    yi = lerped_heights

    Xs += [Xi]
    ys += [yi]

# split data
X = np.concatenate(Xs)
y = np.concatenate(ys)

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2)


"""
Tune and Train Model
"""
# create model
rf = RandomForestRegressor()

# hyperparamter grid search
param_dist = {'n_estimators': np.floor(np.linspace(250, 850, 5)).astype(np.int16),
              'max_depth': np.linspace(10, 30, 5, dtype=np.int8)}

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

# r2 score
r2 = r2_score(y_test, y_pred)
print(f'r2 scoree: {r2}')

# random prediction
ix = np.random.randint(0, len(X_test))
print(f'inp: {X_test[ix]}, predicted: {y_pred[ix]}, truth: {y_test[ix]}')

"""
Save Model
"""