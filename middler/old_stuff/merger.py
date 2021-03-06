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
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from scipy import stats
from scipy.stats import multivariate_normal
from scipy import stats
import time
import math
import datetime
import os
import random


global cam_poses, r2w, r_rtl, focl


""" This class defines a C-like struct """
class PayloadSleipner(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("z", c_float),
                ("p", c_float)]

class PayloadMunin(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("z", c_float)]
                
def set_focal_lengths():

    focl = np.zeros((2,3))

    focl[0,0] = 649.229848
    focl[0,1] = 712.990500
    focl[0,2] = 780.709664
    focl[1,0] = 647.408499
    focl[1,1] = 712.531562
    focl[1,2] = 778.849697

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

''' Translation Matrices'''
def get_transmats(cam_poses):
    
    mat_tran = np.zeros((4,4,3))
    for i in range(3): # Cam 1, 2, 3
        
        cx = cam_poses[i,0]
        cy = cam_poses[i,1]
        cz = cam_poses[i,2]

        # Transformation matrices (translation + rotations around x, y, z)
        mat_tran[:,:,i] = np.array([[1,0,0,cx],
                             [0,1,0,cy],
                             [0,0,1,cz],
                             [0,0,0,1]])
        
    return mat_tran
    
    
    
'''Rotation Matrices'''
def get_rotmats(cam_poses):
    
    mat_rota = np.zeros((3,3,3))
    for i in range(3): # Cam 1, 2, 3
        
        alpha = cam_poses[i,3]
        beta = cam_poses[i,4] 
        gamma = cam_poses[i,5]


        mat_rotx = np.array([[1,0,0],
                             [0,math.cos(alpha), -math.sin(alpha)],
                             [0, math.sin(alpha), math.cos(alpha)]])

        mat_roty = np.array([[math.cos(beta), 0, math.sin(beta)],
                             [0,1,0],
                             [-math.sin(beta), 0, math.cos(beta)]])


        mat_rotz = np.array([[math.cos(gamma), -math.sin(gamma), 0],
                             [math.sin(gamma), math.cos(gamma),0],
                             [0,0,1]])

        # General rotation matrix
        mat_rota[:,:,i] = mat_rotz.dot(mat_roty).dot(mat_rotx)
    
    
    return mat_rota


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
def get_angles_from_pos(obj_pose):
    
    angles = np.zeros(2)
    
    angles[0] = (180/math.pi)*math.atan2((obj_pose[0]),(obj_pose[2])) + 180 # delta_x/delta_z
    angles[1] = (180/math.pi)*math.atan2((obj_pose[1]),(obj_pose[2])) + 180 # delta_y/delta_z

    if(angles[0]>180):
        angles[0] = 360-angles[0]
    if(angles[1]>180):
        angles[1] = 360-angles[1]
    if(angles[0]<-180):
        angles[0] = 360+angles[0]
    if(angles[1]<-180):
        angles[1] = 360+angles[1]

    if(obj_pose[0] < 0):
        angles[0] = -angles[0]
    if(obj_pose[1] < 0):
        angles[1] = -angles[1]

    return angles


''' Create Multivariate Gaussian Distributions'''
def create_mgd(v2r, v_obj_poses):   

    global r2w, r_rtl, ??, ??

    r_?? = np.zeros((3,3))
    r_?? = np.zeros((3,3,3))
    w_?? = np.zeros((3,3))
    w_?? = np.zeros((3,3,3))
    new_?? = np.zeros((4,3)) # including a '1' at the end
    for k in range(3):
        
        # @TODO: only for testing purposes
#         ??[2] = v_obj_poses[2,k]
                                      
        # Rotating Means from virtual-cam space to real-cam space  
        r_??[:,k] = v2r[:,:,k] @ ??
                 
        # Rotating Means from real-cam space to world space 
        w_??[:,k] = r2w[:,:,k] @ r_??[:,k]
    
        # Translating Means from Camera (Real=Virtual) space to World space 
        new_??[:,k] = r_trl[:,:, k] @ [w_??[0,k], w_??[1,k], w_??[2,k],1]                     
                 
        # Rotating Covariance Matrix from virtual-cam space to real-cam space  
        r_??[:,:,k] = v2r[:,:,k] @ ?? @ v2r[:,:,k].T  
                 
        # Rotating Covariance Matrix from real-cam space to world space  
        w_??[:,:,k] = r2w[:,:,k] @ r_??[:,:,k] @ r2w[:,:,k].T 
    
    rv_1 = multivariate_normal(new_??[0:3,0], w_??[:,:,0])
    rv_2 = multivariate_normal(new_??[0:3,1], w_??[:,:,1])
    rv_3 = multivariate_normal(new_??[0:3,2], w_??[:,:,2])
    
    return new_??, w_??, [rv_1, rv_2, rv_3]

def analytical(??, ??, presence, old_p):


    mu = np.zeros(3)
    V_n_p = np.zeros((3,3)) 
    
    if presence[0] == 1:
        V_1 = np.linalg.inv(??[:,:,0])
        V_n_p += V_1
        ??_1 = ??[0:3,0]
    else:
        V_1 = np.zeros((3,3)) 
        ??_1 = np.zeros(3)

    if presence[1] == 1:
        V_2 = np.linalg.inv(??[:,:,1])
        V_n_p += V_2
        ??_2 = ??[0:3,1]
    else:
        V_2 = np.zeros((3,3)) 
        ??_2 = np.zeros(3)

    if presence[2] == 1:
        V_3 = np.linalg.inv(??[:,:,2])
        V_n_p += V_3
        ??_3 = ??[0:3,2]
    else:
        V_3 = np.zeros((3,3)) 
        ??_3 = np.zeros(3)

    if np.sum(presence)>=2:
        V_n =np.linalg.inv(V_n_p)
        mu = ((V_1 @ ??_1) + (V_2 @ ??_2) + (V_3 @ ??_3)) @ V_n

    max_delta = 0.01
    for k in range(3): # for x, y, z
        if mu[k] > old_p[k] + max_delta:
            # print("Jump avoided (up)\n")
            mu[k] = old_p[k] + max_delta
        if mu[k] < old_p[k] - max_delta:
            # print("Jump avoided (down)\n")
            mu[k] = old_p[k] - max_delta

    return mu

##############################################################################################################################
#                                                         UDP SERVER                                                         #
##############################################################################################################################

def udpserver(queue, cam_id):


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
                payload_in = PayloadSleipner.from_buffer_copy(buff)       
                presence = np.random.randint(2)
                queue.put([cam_id, payload_in.x, payload_in.y, payload_in.z, payload_in.p])
                
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


##############################################################################################################################
#                                                           USE DVS                                                          #
##############################################################################################################################

def use_dvs(queue, ip_address, port_nb):

    global focl

    r_obj_poses = np.zeros((3,3))       
    r_obj_angles = np.zeros((2,3))   
    v_obj_poses = np.zeros((3,3))  
    v_obj_angles = np.zeros((2,3))   

    r_?? = np.zeros((3,3))
    r_?? = np.zeros((3,3,3))
    w_?? = np.zeros((3,3))
    w_?? = np.zeros((3,3,3))

    new_?? = np.zeros((4,3)) # including a '1' at the end


    oldsence = np.ones(3)
    presence = np.ones(3)
    prediction = np.ones(3)
    
    server_addr = (ip_address, port_nb)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        s.connect(server_addr)
        print("Connected to {:s}".format(repr(server_addr)))

        counter = 0
        max_counter = 5000
        elapsed = np.zeros(max_counter)
        while(True):

            while not queue.empty():
                datum = queue.get()
                cam_id = datum[0]
                presence[cam_id-1] = datum[4]
                if oldsence[cam_id-1] != presence[cam_id-1]:
                    print("Presence: [{:.3f}, {:.3f}, {:.3f}] ".format(presence[0], presence[1], presence[2]))
                    oldsence[cam_id-1] = presence[cam_id-1]

                px = datum[1]*320
                py = datum[2]*240
                r_obj_angles[:, cam_id-1] = get_angles_from_dvs(px, py, focl, cam_id) 
                
            # print("r_obj_angles [{:.3f}, {:.3f}] [{:.3f}, {:.3f}] [{:.3f}, {:.3f}] ".format(r_obj_angles[0,0], r_obj_angles[1,0], r_obj_angles[0,1], r_obj_angles[1,1], r_obj_angles[0,2], r_obj_angles[1,2]))

            start = datetime.datetime.now()

            # Estimate virtual camera poses (in real camera space)
            v_poses = set_vir_poses(r_obj_angles)
            
            # Get Rotation Matrices: virtual-cam to real-cam 
            v2r = get_rotmats(v_poses)
            
            new_??, w_??, rv = create_mgd(v2r, v_obj_poses)

              
            if np.sum(presence) >= 2:
                # Do predictions
                prediction = analytical(new_??, w_??, presence, prediction)
                # print("Ana. Prediction : [{:.3f}, {:.3f}, {:.3f}]".format(prediction[0], prediction[1], prediction[2]))
           

            stop = datetime.datetime.now()
            diff = stop - start
            elapsed[counter] = int(diff.microseconds)
            if counter < max_counter-1:
                counter += 1
            else:
                print("Elapsed time: " + str(int(np.mean(elapsed))) + " [??s].")
                counter = 0

            payload_out = PayloadMunin(prediction[0], prediction[1], prediction[2])
            nsent = s.send(payload_out)

    except AttributeError as ae:
        print("Error creating the socket: {}".format(ae))
    except socket.error as se:
        print("Exception on socket: {}".format(se))
    finally:
        print("Closing socket")
        s.close()


    
    return 0
  
##############################################################################################################################
#                                                           USE OPT                                                          #
##############################################################################################################################  

def use_opt(queue, ip_address, port_nb):

    r_obj_poses = np.zeros((3,3))       
    r_obj_angles = np.zeros((2,3))   
    v_obj_poses = np.zeros((3,3))  
    v_obj_angles = np.zeros((2,3))   

    r_?? = np.zeros((3,3))
    r_?? = np.zeros((3,3,3))
    w_?? = np.zeros((3,3))
    w_?? = np.zeros((3,3,3))

    new_?? = np.zeros((4,3)) # including a '1' at the end


    oldsence = np.ones(3)
    presence = np.ones(3)
    prediction = np.ones(3)
    
    server_addr = (ip_address, port_nb)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        s.connect(server_addr)
        print("Connected to {:s}".format(repr(server_addr)))

        counter = 0
        max_counter = 5000
        elapsed = np.zeros(max_counter)
        while(True):

            counter += 1
            while not queue.empty():
                datum = queue.get()
                cam_id = datum[0]
                presence[cam_id-1] = datum[4]
                if oldsence[cam_id-1] != presence[cam_id-1]:
                    print("Presence: [{:.3f}, {:.3f}, {:.3f}] ".format(presence[0], presence[1], presence[2]))
                    oldsence[cam_id-1] = presence[cam_id-1]
                

                r_obj_poses[:, cam_id-1] = [datum[1], datum[2], datum[3]]
                r_obj_angles[:, cam_id-1] = get_angles_from_pos(r_obj_poses[:, cam_id-1])
            
            start = datetime.datetime.now()

            # print("r_obj_angles [{:.3f}, {:.3f}] [{:.3f}, {:.3f}] [{:.3f}, {:.3f}] ".format(r_obj_angles[0,0], r_obj_angles[1,0], r_obj_angles[0,1], r_obj_angles[1,1], r_obj_angles[0,2], r_obj_angles[1,2]))

            # Estimate virtual camera poses (in real camera space)
            v_poses = set_vir_poses(r_obj_angles)
            
            # Get Rotation Matrices: virtual-cam to real-cam 
            v2r = get_rotmats(v_poses)
            
            new_??, w_??, rv = create_mgd(v2r, v_obj_poses)

              
            if np.sum(presence) >= 2:
                # Do predictions
                prediction = analytical(new_??, w_??, presence, prediction)
                # print("Ana. Prediction : [{:.3f}, {:.3f}, {:.3f}]".format(prediction[0], prediction[1], prediction[2]))


            stop = datetime.datetime.now()
            diff = stop - start
            elapsed[counter] = int(diff.microseconds)
            if counter < max_counter-1:
                counter += 1
            else:
                print("Elapsed time: " + str(int(np.mean(elapsed))) + " [??s].")
                counter = 0
            payload_out = PayloadMunin(prediction[0], prediction[1], prediction[2])
            nsent = s.send(payload_out)

    except AttributeError as ae:
        print("Error creating the socket: {}".format(ae))
    except socket.error as se:
        print("Exception on socket: {}".format(se))
    finally:
        print("Closing socket")
        s.close()


    
    return 0

    
  

if __name__ == "__main__":
    
    global cam_poses, r2w, r_rtl, ??, ??, focl


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

    # Get camera poses
    cam_poses = set_cam_poses()
    
    # Get Rotation Matrices: real-cam to world 
    r2w = get_rotmats(cam_poses)
    
    # Get Translation Matrices: real-cam to world
    r_trl = get_transmats(cam_poses)


    cam_1 = multiprocessing.Process(target=udpserver, args=(queue,1,))
    cam_2 = multiprocessing.Process(target=udpserver, args=(queue,2,))
    cam_3 = multiprocessing.Process(target=udpserver, args=(queue,3,))

    if d_source == "snn" : 
        # Mean array and covariance matrix in virtual camera space
        ?? = np.array([0,0,-0.75])
        ?? = np.array([[0.2,0,0],[0,0.2,0],[0,0,3.6]])    
        show = multiprocessing.Process(target=use_dvs, args=(queue, ip_address, port_nb))
    if d_source == "opt" :
        # Mean array and covariance matrix in virtual camera space
        ?? = np.array([0,0,-0.9])
        ?? = np.array([[0.02,0,0],[0,0.02,0],[0,0,1.8]])    
        show = multiprocessing.Process(target=use_opt, args=(queue, ip_address, port_nb))

    show.start()
    cam_1.start()
    cam_2.start()
    cam_3.start()

    show.join()
    cam_1.join()
    cam_2.join()
    cam_3.join()

