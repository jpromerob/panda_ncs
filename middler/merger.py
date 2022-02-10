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

global cam_poses, c2w, focl


""" This class defines a C-like struct """
class Payload(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("z", c_float)]
                
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



def udpserver(queue, cam_id):

    global c2w

    port_nb = 3000 + cam_id%3 # cam #1 --> 3001 | cam #2 --> 3002 | cam #3 --> 3000
    server_addr = ('172.16.222.31', port_nb)
    ssock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("Socket created")

    try:
        # bind the server socket and listen
        ssock.bind(server_addr)
        ssock.listen(3)
        print("Listening on port {:d}".format(port_nb))

        while True:
            csock, client_address = ssock.accept()

            buff = csock.recv(512)
            while buff:
                payload_in = Payload.from_buffer_copy(buff)             
                queue.put([cam_id, payload_in.x, payload_in.y, payload_in.z])
                
                buff = csock.recv(512)
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




def conflate(mu, sigma):
    
    mu_1 = mu[0]
    mu_2 = mu[1]
    mu_3 = mu[2]
    
    ss_1 = sigma[0]**2
    ss_2 = sigma[1]**2
    ss_3 = sigma[2]**2
    
    
    mu_conflation = (ss_1*ss_2*mu_3 + ss_2*ss_3*mu_1 + ss_3*ss_1*mu_2)/(ss_3*ss_2 + ss_2*ss_1 + ss_1*ss_3)
    sigma_conflation = math.sqrt((ss_1 * ss_2 * ss_3)/(ss_1*ss_2 + ss_2*ss_3 + ss_3*ss_1))
    
    
    return mu_conflation, sigma_conflation

def get_gaussian(perspective, cam_pdf_params, a2b, b2c):
    
    mu_a = np.zeros((3,4))
    sigma_a = np.zeros((3,4))
    mu_b = np.zeros((3,4))
    sigma_b = np.zeros((3,4))
    mu_c = np.zeros((3,4))
    sigma_c = np.zeros((3,4))
        
    # The gaussians in space 'a' are centered around the values given in space 'a'
    for j in range(3): # x, y, z
        for i in range(3): # Cam 1, 2, 3
            mu_a[j,i] = perspective[j,i] + cam_pdf_params[i,j,0]
            sigma_a[j,i] = cam_pdf_params[i,j,1]    

    # The gaussians are transformed to space 'b' (from space 'a') and their product is calculated (once per axis)
    for j in range(3): # x, y, z
        for i in range(3): # Cam 1, 2, 3
            mu_b[j, i] = a2b[j,0,i]*mu_a[0,i] + a2b[j,1,i]*mu_a[1,i] + a2b[j,2,i]*mu_a[2,i] + a2b[j,3,i]
            sigma_b[j, i] = math.sqrt((a2b[j,0,i]*sigma_a[0,i])**2 +(a2b[j,1,i]*sigma_a[1,i])**2 + (a2b[j,2,i]*sigma_a[2,i])**2)

    # The gaussians are transformed to space 'c' (from space 'b') and their product is calculated (once per axis)
    for j in range(3): # x, y, z
        for i in range(3): # Cam 1, 2, 3
            mu_c[j, i] = b2c[j,0,i]*mu_b[0,i] + b2c[j,1,i]*mu_b[1,i] + b2c[j,2,i]*mu_b[2,i] + b2c[j,3,i]
            sigma_c[j, i] = math.sqrt((b2c[j,0,i]*sigma_b[0,i])**2 +(b2c[j,1,i]*sigma_b[1,i])**2 + (b2c[j,2,i]*sigma_b[2,i])**2)



    for j in range(3): # x, y, z
        mu_c[j, 3], sigma_c[j, 3] = conflate(mu_c[j, 0:3], sigma_c[j, 0:3])

    return mu_c, sigma_c

def merge_stuff(xyz_9, v2c):

    global cam_poses, c2w 
    e_per = np.array([0.02, 0.02, 0.3]) 
    cam_pdf_params = produce_snn_stats(e_per)

    start = time.time()
    mu_w, sigma_w = get_gaussian(xyz_9, cam_pdf_params, v2c, c2w)
    stop = time.time()
    elapsed = stop - start


    xyz_3 = [mu_w[0,3], mu_w[1,3], mu_w[2,3]]

    # print("xyz_9: ({:.3f}, {:.3f}, {:.3f}) | ({:.3f}, {:.3f}, {:.3f}) | ({:.3f}, {:.3f}, {:.3f})".format(xyz_9[0,0], xyz_9[1,0], xyz_9[2,0], xyz_9[0,1], xyz_9[1,1], xyz_9[2,1], xyz_9[0,2], xyz_9[1,2], xyz_9[2,2]))
    print(" ---> xyz_3: ({:.3f}, {:.3f}, {:.3f}) ".format(xyz_3[0], xyz_3[1], xyz_3[2]))

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


def old_use_pixels(queue,):

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
        # print("angles [{:.3f}, {:.3f}] [{:.3f}, {:.3f}] [{:.3f}, {:.3f}] ".format(angles[0,0], angles[1,0], angles[0,1], angles[1,1], angles[0,2], angles[1,2]))

        # poses of the virtual cameras based on angles calculated from pixel positions
        vir_poses = set_vir_poses(angles)

        # transformation matrices
        v2c = get_transmats(vir_poses)
               
        xyz_9[0,cam_id-1] = 0 # x (in camera space)
        xyz_9[1,cam_id-1] = 0 # y (in camera space)
        xyz_9[2,cam_id-1] = -0.7 # z (in camera space)

        
        xyz_3 = merge_stuff(xyz_9, v2c)

    return 0


def use_dvs(queue, ip_address, port_nb):

    global focl

    angles = np.zeros((2,3))
    xyz_9 = np.zeros((3,3))


    server_addr = (ip_address, port_nb)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        s.connect(server_addr)
        print("Connected to {:s}".format(repr(server_addr)))

        counter = 0
        while(True):
            counter += 1
            datum = queue.get()
            cam_id = datum[0]

            px = datum[1]*320
            py = datum[2]*240

            angles[0:2, cam_id-1] = get_angles_from_dvs(px, py, focl, cam_id)
            # print("angles [{:.3f}, {:.3f}] [{:.3f}, {:.3f}] [{:.3f}, {:.3f}] ".format(angles[0,0], angles[1,0], angles[0,1], angles[1,1], angles[0,2], angles[1,2]))

            # poses of the virtual cameras based on angles calculated from pixel positions
            vir_poses = set_vir_poses(angles)

            # transformation matrices
            v2c = get_transmats(vir_poses)
                
            xyz_9[0,cam_id-1] = 0 # x (in camera space)
            xyz_9[1,cam_id-1] = 0 # y (in camera space)
            xyz_9[2,cam_id-1] = -1.0 # z (in camera space)

            
            xyz_3 = merge_stuff(xyz_9, v2c)

            payload_out = Payload(xyz_3[0], xyz_3[1], xyz_3[2])
            nsent = s.send(payload_out)

    except AttributeError as ae:
        print("Error creating the socket: {}".format(ae))
    except socket.error as se:
        print("Exception on socket: {}".format(se))
    finally:
        print("Closing socket")
        s.close()
    
    return 0


'''
This function defines object pose from camera perspective
'''
def define_object_pose(a2b, ground_truth):
    

    perspective = np.zeros((4,1)) # coordinates|cameras
    # Checking output of each camera
    b2a = np.linalg.inv(a2b)
    perspective = b2a.dot(ground_truth)

    return perspective

def use_xyz(queue, ip_address, port_nb):

    angles = np.zeros((2,3))
    xyz_9 = np.zeros((3,3))
    
    server_addr = (ip_address, port_nb)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        s.connect(server_addr)
        print("Connected to {:s}".format(repr(server_addr)))

        counter = 0
        while(True):
            counter += 1
            datum = queue.get()
            cam_id = datum[0]

            x = np.array(datum[1])
            y = np.array(datum[2])
            z = np.array(datum[3])

            angles[0:2, cam_id-1] = get_angles_from_opt(x, y, z)

            # poses of the virtual cameras based on angles calculated from pixel positions
            vir_poses = set_vir_poses(angles)

            # transformation matrices
            v2c = get_transmats(vir_poses)
                
            xyz_9[0,cam_id-1] = 0 # x (in camera space)
            xyz_9[1,cam_id-1] = 0 # y (in camera space)
            xyz_9[2,cam_id-1] = -1.0 # z (in camera space)

            
            xyz_3 = merge_stuff(xyz_9, v2c)

            payload_out = Payload(xyz_3[0], xyz_3[1], xyz_3[2])
            nsent = s.send(payload_out)

    except AttributeError as ae:
        print("Error creating the socket: {}".format(ae))
    except socket.error as se:
        print("Exception on socket: {}".format(se))
    finally:
        print("Closing socket")
        s.close()


    
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


    try:
        d_source = sys.argv[1]
        if d_source != "snn" and d_source != "opt":
            print("Invalid data source")
            quit()
        else:
            print("Valid data source!")

        d_destin = sys.argv[2]
        if d_destin != "nuc" and d_destin != "munin":
            print("Invalid data destination")
            quit()
        else:
            print("Valid data destination!")
            port_nb = 2600
            if d_destin == "nuc" : 
                ip_address = "172.16.222.46"
            if d_destin == "munin" : 
                ip_address = "172.16.222.31"

    except:
        print("Try python3 merger.py <snn|opt> <nuc|munin>")
        quit()


    queue = multiprocessing.Queue()

    focl = set_focal_lengths()

    cam_poses = set_cam_poses()
    c2w = get_transmats(cam_poses)

    cam_1 = multiprocessing.Process(target=udpserver, args=(queue,1,))
    cam_2 = multiprocessing.Process(target=udpserver, args=(queue,2,))
    cam_3 = multiprocessing.Process(target=udpserver, args=(queue,3,))


    if d_source == "snn" : 
        show = multiprocessing.Process(target=use_dvs, args=(queue, ip_address, port_nb))
    if d_source == "opt" :
        show = multiprocessing.Process(target=use_xyz, args=(queue, ip_address, port_nb))

    show.start()
    cam_1.start()
    cam_2.start()
    cam_3.start()

    show.join()
    cam_1.join()
    cam_2.join()
    cam_3.join()

