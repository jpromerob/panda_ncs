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

from coremerge import get_transmats, get_world_gaussian
from snn import define_object_pose, produce_snn_stats
from visuals import plot_gaussians, visualize_3d
from utils import data2text, generate_pdfs

global cam_poses, c2w, focl

def set_focal_lengths():

    focl = np.zeros((2,3))


    focl[0,0] = 649.229848
    focl[0,1] = 712.990500
    focl[0,2] = 810.749526
    focl[1,0] = 647.408499
    focl[1,1] = 712.531562
    focl[1,2] = 804.994749

    return focl

# Poses of virtual cameras with respect to their corresponding real camera space
def set_vir_poses(angles):
   
    vir_poses = np.zeros((3,6))

    # Cam 1
    vir_poses[0,0] = 0
    vir_poses[0,1] = 0
    vir_poses[0,2] = 0
    vir_poses[0,3] = (math.pi/180)*(angles[1,0])    # around X axis -->  ang(YZ)
    vir_poses[0,4] = (math.pi/180)*(-angles[0,0])   # around Y axis --> -ang(XZ)
    vir_poses[0,5] = 0 

    # Cam 2
    vir_poses[1,0] = 0
    vir_poses[1,1] = 0
    vir_poses[1,2] = 0
    vir_poses[1,3] = (math.pi/180)*(angles[1,1])    # around X axis -->  ang(YZ)
    vir_poses[1,4] = (math.pi/180)*(-angles[0,1])   # around Y axis --> -ang(XZ)
    vir_poses[1,5] = 0

    # Cam 3
    vir_poses[2,0] = 0
    vir_poses[2,1] = 0
    vir_poses[2,2] = 0
    vir_poses[2,3] = (math.pi/180)*(angles[1,2])    # around X axis -->  ang(YZ)
    vir_poses[2,4] = (math.pi/180)*(-angles[0,2])   # around Y axis --> -ang(XZ)
    vir_poses[2,5] = 0 

    return vir_poses
    

'''
This function sets the camera poses based on manual readings from optitrack (using camera marker 'hat')
'''
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


'''
This function returns the matrix 'c2w' that converts coordinates in camera space to coordinates in world space.
'''
def get_transmats(cam_poses):
    
    c2w = np.zeros((4,4,3))
    for i in range(3): # Cam 1, 2, 3
        
        cx = cam_poses[i,0]
        cy = cam_poses[i,1]
        cz = cam_poses[i,2]
        alpha = cam_poses[i,3]
        beta = cam_poses[i,4] 
        gamma = cam_poses[i,5]

        # Transformation matrices (translation + rotations around x, y, z)
        mat_tran = np.array([[1,0,0,cx],
                             [0,1,0,cy],
                             [0,0,1,cz],
                             [0,0,0,1]])

        mat_rotx = np.array([[1,0,0,0],
                             [0,math.cos(alpha), -math.sin(alpha),0],
                             [0, math.sin(alpha), math.cos(alpha),0],
                             [0,0,0,1]])

        mat_roty = np.array([[math.cos(beta), 0, math.sin(beta),0],
                             [0,1,0,0],
                             [-math.sin(beta), 0, math.cos(beta),0],
                             [0,0,0,1]])


        mat_rotz = np.array([[math.cos(gamma), -math.sin(gamma), 0, 0],
                             [math.sin(gamma), math.cos(gamma),0, 0],
                             [0,0,1,0],
                             [0,0,0,1]])

        # General transformation matrix 'camera to world' (c2w)
        c2w[:,:,i] = mat_tran.dot(mat_rotz).dot(mat_roty).dot(mat_rotx)
    
    
    return c2w


""" This class defines a C-like struct """
class Payload(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("z", c_float)]


def udpserver(queue, cam_id):

    global c2w

    port_nb = 3000 + cam_id%3 # cam #1 --> 3001 | cam #2 --> 3002 | cam #3 --> 3000
    server_addr = ('172.16.222.31', port_nb)
    ssock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("Socket created")

    try:
        # bind the server socket and listen
        ssock.bind(server_addr)
        print("Bind done")
        ssock.listen(3)
        print("Server listening on port {:d}".format(port_nb))

        while True:
            csock, client_address = ssock.accept()
            # print("Accepted connection from {:s}".format(client_address[0]))

            buff = csock.recv(512)
            while buff:
                payload_in = Payload.from_buffer_copy(buff)             
                queue.put([cam_id, payload_in.x, payload_in.y, payload_in.z])
                # queue.put([cam_id, payload_in.x, payload_in.y, payload_in.z, payload_in.a, payload_in.b])
                
                buff = csock.recv(512)

            # print("Closing connection to client")
            # print("----------------------------")
            csock.close()

    except AttributeError as ae:
        print("Error creating the socket: {}".format(ae))
    except socket.error as se:
        print("Exception on socket: {}".format(se))
    except KeyboardInterrupt:
        ssock.close()
    finally:
        print("Closing socket")
        ssock.close()

def merge_stuff(xyz_9):

    global cam_poses, c2w 
    e_per = np.array([0.02, 0.02, 0.02]) 
    cam_pdf_params = produce_snn_stats(e_per)

    start = time.time()
    mu_c, sigma_c, mu_w, sigma_w = get_world_gaussian(xyz_9, cam_pdf_params, c2w)
    stop = time.time()
    elapsed = stop - start


    xyz_3 = [mu_w[0,3], mu_w[1,3], mu_w[2,3]]

    print("({:.3f}, {:.3f}, {:.3f}) | ({:.3f}, {:.3f}, {:.3f}) | ({:.3f}, {:.3f}, {:.3f})".format(xyz_9[0,0], xyz_9[1,0], xyz_9[2,0], xyz_9[0,1], xyz_9[1,1], xyz_9[2,1], xyz_9[0,2], xyz_9[1,2], xyz_9[2,2]))
    print(" ---> ({:.3f}, {:.3f}, {:.3f}) ".format(xyz_3[0], xyz_3[1], xyz_3[2]))

    return xyz_3


'''
This functions determines the angular 'distance' between camera and object in planez XZ and YZ
'''
def get_angles_from_dvs(px, py, focl, cam_id):

    angles = np.zeros(2)
    
    angles[0] = (180/math.pi)*math.atan2(px, focl[0,cam_id-1]) 
    angles[1] = (180/math.pi)*math.atan2(py, focl[1,cam_id-1]) 

    return angles

'''
This functions determines the angular 'distance' between camera and object in planez XZ and YZ
'''
def get_angles_from_opt(x, y, z):
    angles = np.zeros(2)
    angles[0] = (180/math.pi)*math.atan2(x,z) + 180 # delta_x/delta_z
    angles[1] = (180/math.pi)*math.atan2(y,z) + 180 # delta_y/delta_z

    if(angles[0]>180):
        angles[0] = 360-angles[0]
    if(angles[1]>180):
        angles[1] = 360-angles[1]
    if(angles[0]<-180):
        angles[0] = 360+angles[0]
    if(angles[1]<-180):
        angles[1] = 360+angles[1]

    if(x < 0):
        angles[0] = -angles[0]
    if(y < 0):
        angles[1] = -angles[1]

    return angles


def use_pixels(queue,):

    global focl

    angles = np.zeros((2,3))
    xyz_9 = np.zeros((3,3))

    counter = 0
    while(True):
        counter += 1
        datum = queue.get()
        cam_id = datum[0]

        px = datum[1]*320
        py = datum[2]*240

        angles[0:2, cam_id-1] = get_angles_from_dvs(px, py, focl, cam_id)

        # poses of the virtual cameras based on angles calculated from pixel positions
        vir_poses = set_vir_poses(angles)

        # transformation matrices
        v2c = get_transmats(vir_poses)

        # The virtual camera 'thinks' that the object is located in the center of the image at a distance Z=0.7
        vp = np.array([0, 0, 0.7, 1])

        # transformation from virtual camera space into real camera space
        cp = v2c[:,:,cam_id-1].dot(vp)

        # transformation from real camera space to world space
        gt = c2w[:,:,cam_id-1].dot(cp)

        
        xyz_9[0,cam_id-1] = gt[0] # x (in camera space)
        xyz_9[1,cam_id-1] = gt[1] # y (in camera space)
        xyz_9[2,cam_id-1] = gt[2] # z (in camera space)
        
        xyz_3 = merge_stuff(xyz_9)

    
    return 0


def use_xyz(queue):

    angles = np.zeros((2,3))
    xyz_9 = np.zeros((3,3))
    
    counter = 0
    while(True):
        counter += 1
        datum = queue.get()
        cam_id = datum[0]

        x = np.array(datum[1])
        y = np.array(datum[2])
        z = np.array(datum[3])

        angles[0:2, cam_id-1] = get_angles_from_opt(x, y, z)
        print("angles [{:.3f}, {:.3f}] [{:.3f}, {:.3f}] [{:.3f}, {:.3f}] ".format(angles[0,0], angles[1,0], angles[0,1], angles[1,1], angles[0,2], angles[1,2]))

        # Go from angles to (x, y, z) again
        new_z = z*1
        new_x = new_z*math.tan(-angles[0, cam_id-1]*1*math.pi/180)
        new_y = new_z*math.tan(-angles[1, cam_id-1]*1*math.pi/180)

        # poses of the virtual cameras based on angles calculated from pixel positions
        vir_poses = set_vir_poses(np.zeros((2,3))) 
        # vir_poses = set_vir_poses(angles)

        # transformation matrices
        v2c = get_transmats(vir_poses)

        # The virtual camera 'thinks' that the object is located in the center of the image at a distance Z=0.7
        vp = np.array([new_x, new_y, new_z, 1]) 
        # vp = np.array([0, 0, 0.7, 1])

        # transformation from virtual camera space into real camera space
        cp = v2c[:,:,cam_id-1].dot(vp)

        # transformation from real camera space to world space
        gt = cp #c2w[:,:,cam_id-1].dot([x, y, z, 1])

       
        xyz_9[0,cam_id-1] = gt[0] # x (in camera space)
        xyz_9[1,cam_id-1] = gt[1] # y (in camera space)
        xyz_9[2,cam_id-1] = gt[2] # z (in camera space)

        
        xyz_3 = merge_stuff(xyz_9)

    return 0

def use_xyz_direclty(queue):

    xyz_9 = np.zeros((3,3))
    while(True):
        datum = queue.get()
        cam_id = datum[0]

        xyz_9[0,cam_id-1] = np.array(datum[1]) # x (in camera space)
        xyz_9[1,cam_id-1] = np.array(datum[2]) # y (in camera space)
        xyz_9[2,cam_id-1] = np.array(datum[3]) # z (in camera space)
        
        xyz_3 = merge_stuff(xyz_9)

        print("xyz_3 [{:.3f}, {:.3f}, {:.3f}] ".format(xyz_3[0], xyz_3[1], xyz_3[2]))
    
    return 0





if __name__ == "__main__":
    
    global cam_poses, c2w, focl


    queue = multiprocessing.Queue()

    focl = set_focal_lengths()

    cam_poses = set_cam_poses()
    c2w = get_transmats(cam_poses)

    cam_1 = multiprocessing.Process(target=udpserver, args=(queue,1,))
    cam_2 = multiprocessing.Process(target=udpserver, args=(queue,2,))
    cam_3 = multiprocessing.Process(target=udpserver, args=(queue,3,))

    show = multiprocessing.Process(target=use_xyz, args=(queue,))

    show.start()
    cam_1.start()
    cam_2.start()
    cam_3.start()

    show.join()
    cam_1.join()
    cam_2.join()
    cam_3.join()

