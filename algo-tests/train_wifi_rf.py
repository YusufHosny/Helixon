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
from sklearn.model_selection import GridSearchCV, train_test_split
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
X = np.concatenate(Xs[1:])
y = np.concatenate(ys[1:])

X_unseen, y_unseen = Xs[0], ys[0]

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.15)


"""
Tune and Train Model
"""
# create model
rf = RandomForestRegressor(n_estimators=500, max_depth=22)

if GRIDSEARCH:
    # hyperparamter grid search
    param_grid = {'n_estimators': np.floor(np.linspace(250, 800, 5)).astype(np.int16),
                'max_depth': np.linspace(10, 35, 5, dtype=np.int8)}

    # grid search over params
    grid_search = GridSearchCV(rf, 
        param_grid = param_grid,
        verbose=3)

    # train model
    grid_search.fit(X_train, y_train)


    # store best hyperparams and model
    best_rf = grid_search.best_estimator_
    print(f'best hyperparams: {grid_search.best_params_}')

else:
    rf.fit(X_train, y_train)
    best_rf = rf

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


"""
Evaluate on completely unseen dataset (Quantitative)
"""
# run on unseen set
y_pred = best_rf.predict(X_unseen)

# average distance error
ade = np.mean(np.linalg.norm(y_pred - y_unseen, axis=1))
print(f'unseen set distance average distance error: {ade}')

# max distance error
mde = np.linalg.norm(y_pred - y_unseen, axis=1).max()
print(f'unseen set distance max distance error: {mde}')

# mean squared error over distances
msde = np.mean(np.linalg.norm(y_pred - y_unseen, axis=1)**2)
print(f'unseen set distance mean squared distance error: {msde}')

"""
Apply Spiral model approximation
"""
from model.spiral_model import Spiral
spiral_pitch = 4.2 #m
spiral_radius = 8 #m
path_width = 2.4 #m
spiral = Spiral(spiral_pitch, spiral_radius, path_width)

y_approx = np.zeros_like(y_pred)
gamma = .7
for i, yi in enumerate(y_pred):
    y_approx[i] = spiral.closest_point_to(yi) * gamma + yi * (1-gamma)

"""
Analyze error after spiral model approximation (Quantitative)
"""
# average distance error
ade = np.mean(np.linalg.norm(y_approx - y_unseen, axis=1))
print(f'spiral approx set distance average distance error: {ade}')

# max distance error
mde = np.linalg.norm(y_approx - y_unseen, axis=1).max()
print(f'spiral approx set distance max distance error: {mde}')

# mean squared error over distances
msde = np.mean(np.linalg.norm(y_approx - y_unseen, axis=1)**2)
print(f'spiral approx set distance mean squared distance error: {msde}')

"""
Plot Results (Qualitative)
"""
X, Y, Z = 0, 1, 2
fig = plt.figure()
ax = plt.axes(projection='3d')

ax.scatter(y_unseen[:, X], y_unseen[:, Y], y_unseen[:, Z], 'blue')
ax.plot(y_unseen[:, X], y_unseen[:, Y], y_unseen[:, Z], color=(0., 0., 1., 0.3), linestyle='--')

ax.scatter(y_approx[:, X], y_approx[:, Y], y_approx[:, Z], 'red')
ax.plot(y_approx[:, X], y_approx[:, Y], y_approx[:, Z], color=(1., 0., 0., 0.3), linestyle='--')

ax.scatter(y_pred[:, X], y_pred[:, Y], y_pred[:, Z], 'gray')
ax.plot(y_pred[:, X], y_pred[:, Y], y_pred[:, Z], color=(1., 0., 0., 0.3), linestyle='--')

plt.show()

"""
Save Model
"""
if SAVEMODEL:
    from pickle import dump
    with open(os.path.join("model", "wifi_model.pkl"), "wb") as f:
        dump(best_rf, f, protocol=5)
    with open(os.path.join("model", "bssid_map.pkl"), "wb") as f:
            dump(bssidMap, f, protocol=5)