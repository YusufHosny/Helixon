import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from hlxon_hdf5io import *

raw_timestamp, raw_9dof, raw_rpy, raw_bno, raw_bmp, raw_pressure, gt_timestamp, gt_position, gt_orientation = readHDF5('spiral1')

time = np.array(gt_timestamp)
orient = np.array(gt_orientation)
pos = np.array(gt_position)
pos_x = pos.T[0].T
pos_y = pos.T[1].T
pos_z = pos.T[2].T

fig = plt.figure()
ax = fig.add_subplot(projection="3d")

ax.set(xlim3d=(min(pos_x), max(pos_x)), xlabel='X')
ax.set(ylim3d=(min(pos_y), max(pos_y)), ylabel='Y')
ax.set(zlim3d=(min(pos_z), max(pos_z)), zlabel='Z')

line, = ax.plot([], [], [], lw=2)

def update_lines(i, pos=pos, line=line): 
    line.set_data(pos_x[:i], pos_y[:i])
    line.set_3d_properties(pos_z[:i])
    return line,


ani = animation.FuncAnimation(
    fig, update_lines, len(pos_x), interval=50, blit=True)

plt.show()