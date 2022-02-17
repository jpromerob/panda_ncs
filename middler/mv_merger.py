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


''' This function sets standard deviations in virtual camera space'''
# @TODO: to be removed!
def produce_snn_stats(e_per):
       
    
    cam_pdf_params = np.zeros((3,3,2)) # {1,2,3} | {x,y,z} | {mean, std}
    

    #Cam 1
    cam_pdf_params[0,0,:] = [0, e_per[0]] # x
    cam_pdf_params[0,1,:] = [0, e_per[1]] # y
    cam_pdf_params[0,2,:] = [0, e_per[2]] # z

    #Cam 2
    cam_pdf_params[1,0,:] = [0, e_per[0]] # x
    cam_pdf_params[1,1,:] = [0, e_per[1]] # y
    cam_pdf_params[1,2,:] = [0, e_per[2]] # z

    #Cam 3
    cam_pdf_params[2,0,:] = [0, e_per[0]] # x
    cam_pdf_params[2,1,:] = [0, e_per[1]] # y
    cam_pdf_params[2,2,:] = [0, e_per[2]] # z
    
    
    return cam_pdf_params


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
This function defines object pose in B space starting from A space
It performs a full coordinate transformation (rotation and translation)
b2a is rotation matrix (3x3)
trl is translation matrix (4x4)
'''
def get_b_pose_from_a_pose(b2a, trl, a_pose):
        
    mat = np.zeros((4,4))
    b_pose = np.zeros(4) # coordinates|cameras
            
    mat [3,:] = trl[3,:]
    mat [:,3] = trl[:,3]
    mat[0:3, 0:3] = b2a[0:3,0:3]
        
    a2b = np.linalg.inv(mat)
    b_pose = a2b.dot(np.array([a_pose[0], a_pose[1], a_pose[2], 1]))
    
#     print("b_pose")
#     print(np.round(b_pose[0:3],3))

    return b_pose[0:3]



'''
This function defines object pose from camera perspective
'''
# @TODO: to be removed!
def define_object_pose(a2b, ground_truth):
    

    perspective = np.zeros((4,1)) # coordinates|cameras
    # Checking output of each camera
    b2a = np.linalg.inv(a2b)
    perspective = b2a.dot(ground_truth)

    return perspective

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


# @TODO: to be removed?
def get_joint_rv(x, y, z, rv):
    
    p = rv.pdf([x, y, z])
    
        
    return p
 # @TODO: to be removed?
def get_3_joint_rv(x, y, z, rv):
    
    p1 = rv[0].pdf([x, y, z])
    p2 = rv[1].pdf([x, y, z])
    p3 = rv[2].pdf([x, y, z])
    
    p = np.cbrt(p1*p2*p3)
    
#     p = p1*p2*p3
        
    return p


def get_euclidian_d(w_pose, prediction):
    dx = abs(w_pose[0]-prediction[0])
    dy = abs(w_pose[1]-prediction[1])
    dz = abs(w_pose[2]-prediction[2])
    
    print("dx = {:.3f} | dy = {:.3f} | dz = {:.3f} in cm".format(100*dx, 100*dy, 100*dz))
    
    d = math.sqrt(dx*dx+dy*dy+dz*dz)
    return d

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


''' Create Multivariate Gaussian Distributions'''
def create_mgd(v2r, v_obj_poses):   

    global r2w, r_rtl, μ, Σ

    r_μ = np.zeros((3,3))
    r_Σ = np.zeros((3,3,3))
    w_μ = np.zeros((3,3))
    w_Σ = np.zeros((3,3,3))
    new_μ = np.zeros((4,3)) # including a '1' at the end
    for k in range(3):
        
        # @TODO: only for testing purposes
#         μ[2] = v_obj_poses[2,k]
                                      
        # Rotating Means from virtual-cam space to real-cam space  
        r_μ[:,k] = v2r[:,:,k] @ μ
                 
        # Rotating Means from real-cam space to world space 
        w_μ[:,k] = r2w[:,:,k] @ r_μ[:,k]
    
        # Translating Means from Camera (Real=Virtual) space to World space 
        new_μ[:,k] = r_trl[:,:, k] @ [w_μ[0,k], w_μ[1,k], w_μ[2,k],1]                     
                 
        # Rotating Covariance Matrix from virtual-cam space to real-cam space  
        r_Σ[:,:,k] = v2r[:,:,k] @ Σ @ v2r[:,:,k].T  
                 
        # Rotating Covariance Matrix from real-cam space to world space  
        w_Σ[:,:,k] = r2w[:,:,k] @ r_Σ[:,:,k] @ r2w[:,:,k].T 
    
    rv_1 = multivariate_normal(new_μ[0:3,0], w_Σ[:,:,0])
    rv_2 = multivariate_normal(new_μ[0:3,1], w_Σ[:,:,1])
    rv_3 = multivariate_normal(new_μ[0:3,2], w_Σ[:,:,2])
    
    return new_μ, w_Σ, [rv_1, rv_2, rv_3]


def predict_pose(rv, mean, presence):
    
        
    nb_pts = 11
    diff = 0.5 # 50[cm]


    # use 'mean' only from cameras with 'presence'
    idx_pre = presence > 0 
    useful_mean = mean[:,idx_pre]
    x_0 = np.mean(useful_mean[0,:])
    y_0 = np.mean(useful_mean[1,:])
    z_0 = np.mean(useful_mean[2,:])
    
    count = 0

    while True:

        nb_pts = nb_pts*(1+count)-count

        lims = np.array([[x_0-diff, x_0+diff],[y_0-diff, y_0+diff],[z_0-diff, z_0+diff]])   

        x = np.linspace(lims[0,0], lims[0,1], num=nb_pts) 
        y = np.linspace(lims[1,0], lims[1,1], num=nb_pts)
        z = np.linspace(lims[2,0], lims[2,1], num=nb_pts)

        xx, yy, zz = np.meshgrid(x, y, z, indexing='ij')
    
        xyz = np.zeros((nb_pts,nb_pts, nb_pts,3))
        xyz[:,:,:,0] = xx
        xyz[:,:,:,1] = yy
        xyz[:,:,:,2] = zz

        # Getting joint probabilities
        start = datetime.datetime.now()

        if presence[0] > 0:
            p1 = rv[0].pdf(xyz)
        else:
            p1 = 1

        if presence[1] > 0:
            p2 = rv[1].pdf(xyz)
        else:
            p2 = 1

        if presence[2] > 0:
            p3 = rv[2].pdf(xyz)
        else:
            p3 = 1



        p = np.cbrt(p1*p2*p3)


        stop = datetime.datetime.now()
        elapsed = stop - start

        # Indices of Max Probability
        imp = np.unravel_index(np.argmax(p, axis=None), p.shape) 

        x_0 = x[imp[0]]
        y_0 = y[imp[1]]
        z_0 = z[imp[2]]

        
        count+= 1     
        diff = 2*diff/(nb_pts-3) # 5cm
        if count > 1:
            # print("Joint probabilities obtained after: " + str(int(elapsed.microseconds/1000)) + " [ms].")
            # print("Prediction: ({:.3f}, {:.3f}, {:.3f})".format(x[imp[0]], y[imp[1]], z[imp[2]]))
            break

    prediction = np.array([x_0, y_0, z_0])
    
    return prediction

def use_dvs(queue, ip_address, port_nb):

    global focl

    r_obj_poses = np.zeros((3,3))       
    r_obj_angles = np.zeros((2,3))   
    v_obj_poses = np.zeros((3,3))  
    v_obj_angles = np.zeros((2,3))   

    r_μ = np.zeros((3,3))
    r_Σ = np.zeros((3,3,3))
    w_μ = np.zeros((3,3))
    w_Σ = np.zeros((3,3,3))

    new_μ = np.zeros((4,3)) # including a '1' at the end


    presence = np.ones(3)
    prediction = np.ones(3)
    
    server_addr = (ip_address, port_nb)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        s.connect(server_addr)
        print("Connected to {:s}".format(repr(server_addr)))

        counter = 0
        while(True):
            counter += 1
            while not queue.empty():
                datum = queue.get()
                cam_id = datum[0]
                presence[cam_id-1] = datum[4]

                px = datum[1]*320
                py = datum[2]*240
                r_obj_angles[:, cam_id-1] = get_angles_from_dvs(px, py, focl, cam_id) 
            # print("r_obj_angles [{:.3f}, {:.3f}] [{:.3f}, {:.3f}] [{:.3f}, {:.3f}] ".format(r_obj_angles[0,0], r_obj_angles[1,0], r_obj_angles[0,1], r_obj_angles[1,1], r_obj_angles[0,2], r_obj_angles[1,2]))

            # Estimate virtual camera poses (in real camera space)
            v_poses = set_vir_poses(r_obj_angles)
            
            # Get Rotation Matrices: virtual-cam to real-cam 
            v2r = get_rotmats(v_poses)

            # @ THIS IS NOT NECESSARY BUT GOOD FOR TESTING
            # Estimate object poses (in virtual camera space)
            # for k in range(3):
            #     v_obj_poses[:,k] = get_b_pose_from_a_pose(v2r[:,:,k], np.identity(4), r_obj_poses[:,k])
            #     v_obj_angles[0:2, k] = get_angles_from_pos(v_obj_poses[:,k])
            #     # @TODO: The angles here should be exactly ZERO !!! WTF?!?!?
            #     # print(np.round(v_poses[:,k],3))        
            #     # print(v_angles[0:2, k])
            # print("vir_z_123 = ({:.3f}, {:.3f}, {:.3f})".format(np.round(v_obj_poses[2,0],3), np.round(v_obj_poses[2,1],3), np.round(v_obj_poses[2,2],3)))
    
            
            new_μ, w_Σ, rv = create_mgd(v2r, v_obj_poses)

              
            print("Presence: [{:.3f}, {:.3f}, {:.3f}] ".format(presence[0], presence[1], presence[2]))
            if np.sum(presence) >= 2:
                # Do predictions
                prediction = predict_pose(rv, new_μ, presence)
                # mu_conflation = conflate3D(new_μ[0:3,:], w_Σ, presence)
                # print("P : [{:.3f}, {:.3f}, {:.3f}] vs C : [{:.3f}, {:.3f}, {:.3f}] ".format(prediction[0], prediction[1], prediction[2], mu_conflation[0], mu_conflation[1], mu_conflation[2]))
           

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

def conflate1D(μ, Σ, presence):
    

    mu = np.zeros(3)
    sigma_conflation = np.zeros(3)
    sigma = np.zeros(4)

    for k in range(3): # for x, y, z
        if presence[0] == 1:
            mu_1 = μ[k,0]
            s_1 = Σ[k,k,0]**2
        else:
            mu_1 = np.zeros(3)
            s_1 = np.identity(3)

        if presence[1] == 1:
            mu_2 = μ[k,1]
            s_2 = Σ[k,k,1]**2
        else:
            mu_2 = np.zeros(3)
            s_2 = np.identity(3)

        if presence[2] == 1:
            mu_3 = μ[k,2]
            s_3 = Σ[k,k,2]**2
        else:
            mu_3 = np.zeros(3)
            s_3 = np.identity(3) 
    
        mu[k] = (s_1*s_2*mu_3 + s_2*s_3*mu_1 + s_3*s_1*mu_2)/(s_3*s_2 + s_2*s_1 + s_1*s_3)
    
    return mu

def conflate3D(μ, Σ, presence):
    
    if presence[0] == 1:
        mu_1 = μ[:,0]
        s_1 = Σ[:,:,0]
    else:
        mu_1 = np.zeros(3)
        s_1 = np.identity(3)

    if presence[1] == 1:
        mu_2 = μ[:,1]
        s_2 = Σ[:,:,1]
    else:
        mu_2 = np.zeros(3)
        s_2 = np.identity(3)

    if presence[2] == 1:
        mu_3 = μ[:,2]
        s_3 = Σ[:,:,2]
    else:
        mu_3 = np.zeros(3)
        s_3 = np.identity(3)
        

    num = ((s_1 * s_2) @ mu_3 ) + ((s_2 * s_3) @ mu_1) + ((s_1 * s_3) @ mu_2)
    den = (s_2 * s_3) + (s_1 * s_2) + (s_1* s_3)


    n_d = np.linalg.inv(den) @ num

    mu_conflation = n_d

    return mu_conflation
    

def use_xyz(queue, ip_address, port_nb):

    r_obj_poses = np.zeros((3,3))       
    r_obj_angles = np.zeros((2,3))   
    v_obj_poses = np.zeros((3,3))  
    v_obj_angles = np.zeros((2,3))   

    r_μ = np.zeros((3,3))
    r_Σ = np.zeros((3,3,3))
    w_μ = np.zeros((3,3))
    w_Σ = np.zeros((3,3,3))

    new_μ = np.zeros((4,3)) # including a '1' at the end


    presence = np.ones(3)
    prediction = np.ones(3)
    
    server_addr = (ip_address, port_nb)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        s.connect(server_addr)
        print("Connected to {:s}".format(repr(server_addr)))

        counter = 0
        while(True):
            counter += 1
            while not queue.empty():
                datum = queue.get()
                cam_id = datum[0]
                presence[cam_id-1] = datum[4]

                r_obj_poses[:, cam_id-1] = [datum[1], datum[2], datum[3]]
                r_obj_angles[:, cam_id-1] = get_angles_from_pos(r_obj_poses[:, cam_id-1])
            # print("r_obj_angles [{:.3f}, {:.3f}] [{:.3f}, {:.3f}] [{:.3f}, {:.3f}] ".format(r_obj_angles[0,0], r_obj_angles[1,0], r_obj_angles[0,1], r_obj_angles[1,1], r_obj_angles[0,2], r_obj_angles[1,2]))

            # Estimate virtual camera poses (in real camera space)
            v_poses = set_vir_poses(r_obj_angles)
            
            # Get Rotation Matrices: virtual-cam to real-cam 
            v2r = get_rotmats(v_poses)

            # @ THIS IS NOT NECESSARY BUT GOOD FOR TESTING
            # Estimate object poses (in virtual camera space)
            # for k in range(3):
            #     v_obj_poses[:,k] = get_b_pose_from_a_pose(v2r[:,:,k], np.identity(4), r_obj_poses[:,k])
            #     v_obj_angles[0:2, k] = get_angles_from_pos(v_obj_poses[:,k])
            #     # @TODO: The angles here should be exactly ZERO !!! WTF?!?!?
            #     # print(np.round(v_poses[:,k],3))        
            #     # print(v_angles[0:2, k])
            # print("vir_z_123 = ({:.3f}, {:.3f}, {:.3f})".format(np.round(v_obj_poses[2,0],3), np.round(v_obj_poses[2,1],3), np.round(v_obj_poses[2,2],3)))
    
            
            new_μ, w_Σ, rv = create_mgd(v2r, v_obj_poses)

              
            print("Presence: [{:.3f}, {:.3f}, {:.3f}] ".format(presence[0], presence[1], presence[2]))
            if np.sum(presence) >= 2:
                # Do predictions
                prediction = predict_pose(rv, new_μ, presence)
                # mu_conflation = conflate1D(new_μ[0:3,:], w_Σ, presence)
                # print("P : [{:.3f}, {:.3f}, {:.3f}] vs C : [{:.3f}, {:.3f}, {:.3f}] ".format(prediction[0], prediction[1], prediction[2], mu_conflation[0], mu_conflation[1], mu_conflation[2]))

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
    
    global cam_poses, r2w, r_rtl, μ, Σ, focl


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
        μ = np.array([0,0,-0.75])
        Σ = np.array([[0.2,0,0],[0,0.2,0],[0,0,2]])    
        show = multiprocessing.Process(target=use_dvs, args=(queue, ip_address, port_nb))
    if d_source == "opt" :
        # Mean array and covariance matrix in virtual camera space
        μ = np.array([0,0,-0.9])
        Σ = np.array([[0.02,0,0],[0,0.02,0],[0,0,1.8]])    
        show = multiprocessing.Process(target=use_xyz, args=(queue, ip_address, port_nb))

    show.start()
    cam_1.start()
    cam_2.start()
    cam_3.start()

    show.join()
    cam_1.join()
    cam_2.join()
    cam_3.join()

