
import socket
from ctypes import *
import numpy as np
import multiprocessing 

import time
import math
import datetime
import struct 
import pdb
from geometry import *
from gaussians import *

x = -0.32270892
y = 0.02973045
z = -0.32270892
alpha = -3.1026893
beta = -1.20907684
gamma = 3.08738275

centroid_poses = np.zeros((1,6))
centroid_poses[0,0] = x # x
centroid_poses[0,1] = y # y
centroid_poses[0,2] = x # z
centroid_poses[0,3] = alpha # alpha
centroid_poses[0,4] = beta  # beta
centroid_poses[0,5] = gamma # gamma


vertex_poses = np.zeros(4)
vertex_poses[0] = -0.05 # x
vertex_poses[1] =  0.00 # y
vertex_poses[2] =  0.12 # z
vertex_poses[3] = 1 # just like that

if __name__ == "__main__":
    
    r2w = get_rotmats(centroid_poses)
    trl = get_transmats(centroid_poses)


    c2w = trl[:,:,0].dot(r2w[:,:,0])

    pdb.set_trace()

    vertex_poses = c2w.dot(vertex_poses)
    print(vertex_poses)