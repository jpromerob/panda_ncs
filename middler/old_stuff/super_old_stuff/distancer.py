#!/usr/bin/env python3

""" server.py - Echo server for sending/receiving C-like structs via socket
References:
- Ctypes: https://docs.python.org/3/library/ctypes.html
- Sockets: https://docs.python.org/3/library/socket.html
"""

import socket
import sys
import random
from ctypes import *
import numpy as np
import multiprocessing 

import os
import math
import time
import random

from coremerge import get_transmats
from visuals import plot_gaussians, visualize_3d
from snn import produce_snn_stats
from utils import data2text, generate_pdfs

global cam_poses

def set_cam_poses():

    cam_poses = np.zeros((3,6))

    # Cam 1
    cam_poses[0,0] = -0.099 # cam1:cx
    cam_poses[0,1] = 0.968 # cam1:cy
    cam_poses[0,2] = 1.363 # cam1:cz
    cam_poses[0,3] = (math.pi/180)*(-71.499) # cam1:alpha
    cam_poses[0,4] = (math.pi/180)*(16.753) # cam1:beta
    cam_poses[0,5] = (math.pi/180)*(-20.992) # cam1:gamma

    # Cam 2
    cam_poses[1,0] = -0.570 # cam2:cx
    cam_poses[1,1] = 0.970 # cam2:cy
    cam_poses[1,2] = 1.395 # cam2:cz
    cam_poses[1,3] = (math.pi/180)*(-62.113) # cam2:alpha
    cam_poses[1,4] = (math.pi/180)*(-42.374) # cam2:beta
    cam_poses[1,5] = (math.pi/180)*(-6.134) # cam2:gamma

    # Cam 3
    cam_poses[2,0] = -0.664 # cam3:cx
    cam_poses[2,1] =  0.979 # cam3:cy
    cam_poses[2,2] =  0.538 # cam3:cz
    cam_poses[2,3] = (math.pi/180)*(148.698)# cam3:alpha
    cam_poses[2,4] = (math.pi/180)*(-46.056)# cam3:beta
    cam_poses[2,5] = (math.pi/180)*(148.752)# cam3:gamma


    return cam_poses



def get_distance(x_cam, y_cam, z_cam, x_led, y_led, z_led):

    dx = (x_cam-x_led)
    dy = (y_cam-y_led)
    dz = (z_cam-z_led)

    d = math.sqrt(dx*dx + dy*dy + dz*dz)

    return d


if __name__ == "__main__":
    
    global cam_poses

    cam_poses = set_cam_poses()


    led_poses = np.zeros((3,3))

    led_poses[0,:] = [-0.27656, 0.12550, 0.80333]
    led_poses[1,:] = [-0.48727, 0.63966, 1.17090]
    led_poses[2,:] = [-0.46684, 0.40316, 1.19424]



    for cam in [0,1,2]:
        for led in [0,1,2]:
            d = get_distance(cam_poses[cam,0], cam_poses[cam,1], cam_poses[cam,2], led_poses[led,0], led_poses[led,1], led_poses[led,2])
            print("Distance Cam#{:.3f} to LED in take {:.3f} : {:.3f} [m] ".format(cam+1, led+1, d))






