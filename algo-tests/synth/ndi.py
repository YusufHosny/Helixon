from os import path as osp
import numpy as np
import matplotlib.pyplot as plt
import csv
from scipy.spatial.transform import Rotation

# get synth data
synthdata = []
with open(osp.join('data', 'synth', 'synthetic_data.csv')) as csvfile:
    for row in csv.DictReader(csvfile):
        synthdata += [row]

# format synth data
N = len(synthdata)
X, Y, Z = 0, 1, 2 # for convenience
ts = np.zeros((N))
araw = np.zeros((N, 3))
gyro = np.zeros((N, 3))
magn = np.zeros((N, 3))
gtpos = np.zeros((N, 3))
gtrpy = np.zeros((N, 3))
tbno = np.zeros(N)
tbmp = np.zeros(N)
pressure = np.zeros(N)

for i, di in enumerate(synthdata):
    araw[i, X] += float(di['accelx'])
    araw[i, Y] += float(di['accely'])
    araw[i, Z] += float(di['accelz'])

    gyro[i, X] += float(di['gyrox'])
    gyro[i, Y] += float(di['gyroy'])
    gyro[i, Z] += float(di['gyroz'])

    magn[i, X] += float(di['magnx'])
    magn[i, Y] += float(di['magny'])
    magn[i, Z] += float(di['magnz'])

    gtpos[i, X] += float(di['gt_posx'])
    gtpos[i, Y] += float(di['gt_posy'])
    gtpos[i, Z] += float(di['gt_posz'])

    gtrpy[i, X] += float(di['gt_roll'])
    gtrpy[i, Y] += float(di['gt_pitch'])
    gtrpy[i, Z] += float(di['gt_yaw'])

    tbno[i] += float(di['tempbno'])
    tbmp[i] += float(di['tempbmp'])
    pressure[i] += float(di['pressure'])

    ts[i] += float(di['timestamp'])


# rotate acceleration to global coords
accel = np.zeros_like(araw)
for i, rpyi in enumerate(gtrpy):
    rot = Rotation.from_euler('xyz', rpyi, degrees=True).inv()
    accel[i] = rot.apply(araw[i])

# remove gravity vector
accel[:, Z] += 9.81

# dead reckoning ndi on synth
pos = np.zeros((N, 3))
vi = np.array([0., 0., 0.])
for i in range(1, N):
    dt = (ts[i]-ts[i-1])
    vi += accel[i]*dt
    pos[i] = pos[i-1] + vi*dt + 0.5*accel[i]*dt**2
    

# plot synth ndi and gt
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(pos[:, 0], pos[:, 1], pos[:, 2], 'blue')
ax.plot3D(gtpos[:, 0], gtpos[:, 1], gtpos[:, 2], 'gray')
plt.show()
