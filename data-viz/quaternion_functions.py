import bpy 
import h5py
import math 
import numpy as np 
from mathutils import Quaternion as Q

q_rotation = Q((1,0,0,0))

def q_multiply(a:Q, b:Q)->Q:
    qw = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3]
    qx = a[0]*b[1] + a[1]*b[0] - a[2]*b[3] + a[3]*b[2]
    qy = a[0]*b[2] + a[1]*b[3] + a[2]*b[0] - a[3]*b[1]
    qz = a[0]*b[3] - a[1]*b[2] + a[2]*b[1] + a[3]*b[0]
    return Q((qw, qx, qy, qz))

def q_init_rotation(q_0:Q): 
    q_init = Q((1,0, 0, 0))
    q_0.invert()
    global q_rotation 
    q_rotation = qu_multiply(q_init, q_0)
    
def q_rotate(q_i: Q)->Q: 
    global q_rotation
    return q_multiply(q_rotation, q_i)

