#!/usr/bin/env python3

""" server.py - Echo server for sending/receiving C-like structs via socket
References:
- Ctypes: https://docs.python.org/3/library/ctypes.html
- Sockets: https://docs.python.org/3/library/socket.html
"""

import socket
import sys
import signal
import random
from ctypes import *
import numpy as np
import multiprocessing 
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
matplotlib.use("TkAgg")
matplotlib.rcParams['toolbar'] = 'None' 
import matplotlib.pyplot as plt

from scipy import stats
from scipy.stats import multivariate_normal
import matplotlib.animation as animation
from scipy import stats
import time
import math
import datetime
import os
import random
import cv2
import pickle
import struct ## new
import zlib


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

class Ev_frame:
    def __init__(self, cam_id, frame):
        self.cam_id = cam_id
        self.frame = frame

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
def set_vir_poses(angles, v_poses, presence):
   
    for k in range(3): # for cameras 1|2|3
        # Cam k+1
        if presence[k]==1:
            v_poses[k,0] = 0
            v_poses[k,1] = 0
            v_poses[k,2] = 0
            v_poses[k,3] = (math.pi/180)*(angles[1,k])    # around X axis -->  ang(YZ)
            v_poses[k,4] = (math.pi/180)*(-angles[0,k])   # around Y axis --> -ang(XZ)
            v_poses[k,5] = 0 

    return v_poses
    

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

'''  '''
def get_dvs_from_angles(angles, focl, cam_id):

    px = math.tan((angles[0]*math.pi/180))*focl[0,cam_id-1]
    py = math.tan((angles[1]*math.pi/180))*focl[1,cam_id-1]

    return px, py


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

def analytical(μ, Σ, presence, old_p):


    mu = np.zeros(3)
    V_n_p = np.zeros((3,3)) 
    
    if presence[0] == 1:
        V_1 = np.linalg.inv(Σ[:,:,0])
        V_n_p += V_1
        μ_1 = μ[0:3,0]
    else:
        V_1 = np.zeros((3,3)) 
        μ_1 = np.zeros(3)

    if presence[1] == 1:
        V_2 = np.linalg.inv(Σ[:,:,1])
        V_n_p += V_2
        μ_2 = μ[0:3,1]
    else:
        V_2 = np.zeros((3,3)) 
        μ_2 = np.zeros(3)

    if presence[2] == 1:
        V_3 = np.linalg.inv(Σ[:,:,2])
        V_n_p += V_3
        μ_3 = μ[0:3,2]
    else:
        V_3 = np.zeros((3,3)) 
        μ_3 = np.zeros(3)

    if np.sum(presence)>=2:
        V_n =np.linalg.inv(V_n_p)
        mu = ((V_1 @ μ_1) + (V_2 @ μ_2) + (V_3 @ μ_3)) @ V_n

    max_delta = 0.001
    for k in range(3): # for x, y, z
        if mu[k] > old_p[k] + max_delta:
            # print("Jump avoided (up)\n")
            mu[k] = old_p[k] + max_delta
        if mu[k] < old_p[k] - max_delta:
            # print("Jump avoided (down)\n")
            mu[k] = old_p[k] - max_delta


    return mu

##############################################################################################################################
#                                                    POSITION UDP SERVER                                                     #
##############################################################################################################################

def pos_server(merge_queue, cam_id):


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
                merge_queue.put([cam_id, payload_in.x, payload_in.y, payload_in.z, payload_in.p])
                
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
#                                                 EVENT 'FRAME' TCP SERVER                                                   #
##############################################################################################################################
def img_server(bkgrnd_queue, cam_id):

    ip_address = "172.16.222.31"
    port_nb = 4000 + cam_id%3 # cam #1 --> 4001 | cam #2 --> 4002 | cam #3 --> 4000

    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    print('Socket created')

    s.bind((ip_address, port_nb))
    print('Socket bind complete')
    s.listen(10)
    print('Socket now listening')

    conn,addr=s.accept()

    data = b""
    payload_size = struct.calcsize(">L")
    # print("payload_size: {}".format(payload_size))
    while True:
        while len(data) < payload_size:
            # print("Recv: {}".format(len(data)))
            data += conn.recv(4096*4)

        # print("Done Recv: {}".format(len(data)))
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]
        # print("msg_size: {}".format(msg_size))
        while len(data) < msg_size:
            data += conn.recv(4096*4)
        frame_data = data[:msg_size]
        data = data[msg_size:]

        frame=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        
        bkgrnd_queue.put(Ev_frame(cam_id, 120*np.transpose(frame, (1,0,2))))


##############################################################################################################################
#                                                          COMBINER                                                          #
##############################################################################################################################

def combiner(merge_queue, target_queue, xyz_queue, ip_address, port_nb, dvs_is_src):

    global focl, Σ, offset, vis_flag

    v_poses = np.zeros((3,6))

    r_obj_poses = np.zeros((3,3))       
    r_obj_angles = np.zeros((2,3))   
    v_obj_poses = np.zeros((3,3))  
    v_obj_angles = np.zeros((2,3))   

    r_μ = np.zeros((3,3))
    r_Σ = np.zeros((3,3,3))
    w_μ = np.zeros((3,3))
    w_Σ = np.zeros((3,3,3))

    v_Σ = np.zeros((3,3,3))

    # current Σ in 'virtual camera space' for cams 1|2|3
    for k in range(3):
        v_Σ[:,:,k] = Σ 

    new_μ = np.zeros((4,3)) # including a '1' at the end

    px = np.zeros(3)
    py = np.zeros(3)

    oldsence = np.ones(3)
    presence = np.ones(3)
    prediction = np.ones(3)
    olddiction = np.ones(3)
    
    server_addr = (ip_address, port_nb)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        s.connect(server_addr)
        print("Connected to {:s}".format(repr(server_addr)))

        counter = 0
        max_counter = 5000
        elapsed = np.zeros(max_counter)
        while(True):

            while not merge_queue.empty():
                datum = merge_queue.get()
                cam_id = datum[0]
                presence[cam_id-1] = datum[4]
                if oldsence[cam_id-1] != presence[cam_id-1]:
                    # print("Presence: [{:.3f}, {:.3f}, {:.3f}] ".format(presence[0], presence[1], presence[2]))
                    oldsence[cam_id-1] = presence[cam_id-1]
                
                if dvs_is_src:
                    px[cam_id-1] = datum[1]*320
                    py[cam_id-1] = datum[2]*240
                    r_obj_angles[:, cam_id-1] = get_angles_from_dvs(px[cam_id-1], py[cam_id-1], focl, cam_id) 
                else:
                    r_obj_poses[:, cam_id-1] = [datum[1], datum[2], datum[3]]
                    r_obj_angles[:, cam_id-1] = get_angles_from_pos(r_obj_poses[:, cam_id-1])
                    px[cam_id-1], py[cam_id-1] = get_dvs_from_angles(r_obj_angles[:, cam_id-1], focl, cam_id)
            

            if vis_flag:
                for k in range(3):
                    target_queue.put([k+1, px[k]+320, py[k]+240, presence[k]])

            start = datetime.datetime.now()

            # Estimate virtual camera poses (in real camera space)
            v_poses = set_vir_poses(r_obj_angles, v_poses, presence)
            
            # Get Rotation Matrices: virtual-cam to real-cam 
            v2r = get_rotmats(v_poses)
            
            # Create Multivariate Gaussian Distributions
            new_μ, w_Σ, v_Σ = create_mgd(v2r, v_obj_poses)
              
            # Do predictions
            prediction = analytical(new_μ, w_Σ, presence, prediction)

            # print("Ana. Prediction : [{:.3f}, {:.3f}, {:.3f}]".format(prediction[0], prediction[1], prediction[2]))


            stop = datetime.datetime.now()
            diff = stop - start
            elapsed[counter] = int(diff.microseconds)
            if counter < max_counter-1:
                counter += 1
            else:
                print("Elapsed time: " + str(int(np.mean(elapsed))) + " [μs].")
                counter = 0
            payload_out = PayloadMunin(prediction[0]+offset[0], prediction[1]+offset[1], prediction[2]+offset[2])
            xyz_queue.put([prediction[0]+offset[0], prediction[1]+offset[1], prediction[2]+offset[2]])
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
#                                                          VISUALIZE                                                         #
##############################################################################################################################  

# This function is called periodically from FuncAnimation
def rt_xyz(i, xyz_queue, axs, t, x, y, z, xyz):


    while not xyz_queue.empty():
        xyz = xyz_queue.get(False)

    # Add x and y to lists
    t.append(datetime.datetime.now().strftime('%H:%M:%S.%f'))
    x.append(xyz[0])
    y.append(xyz[1])
    z.append(xyz[2])

    # Limit x and y lists to 100 items
    t = t[-100:]
    x = x[-100:]
    y = y[-100:]
    z = z[-100:]

    txt_x = txt_y = txt_z = "No signal"
    if x[-1] != -100:
        txt_x = "x = {:.3f} [m] ".format(x[-1]) 
    if y[-1] != -100:
        txt_y = "y = {:.3f} [m] ".format(y[-1]) 
    if z[-1] != -100:
        txt_z = "z = {:.3f} [m] ".format(z[-1]) 

    # Draw x and y lists
    axs[0].clear()
    axs[0].plot(t, x, color='r')
    axs[0].text(t[0], 0.1, txt_x, fontsize='xx-large')
    axs[0].xaxis.set_visible(False)
    axs[0].set_ylim([-0.7,0.3])
    axs[0].set_ylabel('x')

    axs[1].clear()
    axs[1].plot(t, y, color='g')
    axs[1].text(t[0], 0.8, txt_y, fontsize='xx-large')
    axs[1].xaxis.set_visible(False)
    axs[1].set_ylim([0,1])
    axs[1].set_ylabel('y')

    axs[2].clear()
    axs[2].plot(t, z, color='b')
    axs[2].text(t[0], 1.3, txt_z, fontsize='xx-large')
    axs[2].xaxis.set_visible(False)
    axs[2].set_ylim([0.5,1.5])
    axs[2].set_ylabel('z')

    axs[0].set_title("Object Position in Workspace")



def oscilloscope(xyz_queue):

    # Create figure for plotting
    fig, axs = plt.subplots(3, figsize=(8, 12))

    t = []
    x = []
    y = []
    z = []

    i = 0
    xyz = [-100,-100,-100]

    # Set up plot to call rt_xyz() function periodically
    ani = animation.FuncAnimation(fig, rt_xyz, fargs=(xyz_queue, axs, t, x, y, z, xyz), interval=1)
    plt.show()

def visualize(target_queue, bkgrnd_queue):

    cam_shape = (480*2+3,640*2+3)

    logo = cv2.imread('NCS_963_1283.png')
    image = logo

    # Black background in all active subplots
    bgi_1 = np.zeros((480,640,3))
    bgi_2 = np.zeros((480,640,3))
    bgi_3 = np.zeros((480,640,3))

    x = [0, 0, 0]
    y = [0, 0, 0]
    presence = [0, 0, 0]

    counter = 0
    while True:

        counter += 1
        white = [255, 255, 255]

        # Update target's (x,y)
        while not target_queue.empty():

            datum = target_queue.get()         
            cam_id = datum[0]
            presence[cam_id-1]=datum[3]

            x_0 = 0
            y_0 = 0

            if cam_id == 1:
                x[0] = int(datum[1])
                y[0] = int(datum[2])
            if cam_id == 2:
                x[1] = int(datum[1])
                y[1] = int(datum[2])
            if cam_id == 3:
                x[2] = int(datum[1])
                y[2] = int(datum[2])

        # Update background
        while not bkgrnd_queue.empty():
            evfr = bkgrnd_queue.get()
            if evfr.cam_id == 1:
                bgi_1 = evfr.frame
            if evfr.cam_id == 2:
                bgi_2 = evfr.frame
            if evfr.cam_id == 3:
                bgi_3 = evfr.frame

        # Draw Backgrounds
        image[480+1:480*2+1, 1:640+1,:] = bgi_1     # Bottom Left:    Cam#1
        image[1:480+1, 1:640+1,:] = bgi_2           # Top left :      Cam#2
        image[1:480+1, 641:640*2+1,:] = bgi_3       # Top right :     Cam#3
        # Draw TargetS
        color = [0, 0, 255]
        diameter = 16
        thickness = 4
        for cam_id in [1,2,3]:

            if cam_id == 1:
                x_0 = 1
                y_0 = 480*2+1
            if cam_id == 2:
                x_0 = 1
                y_0 = 480
            if cam_id == 3:
                x_0 = 641
                y_0 = 480

            if presence[cam_id-1] == 1:
                
                for i in range(diameter):

                    # Horizontal Right
                    if x[cam_id-1]+i <= 640-1 and x[cam_id-1]+i > 0:
                        image[y_0-y[cam_id-1],x_0+x[cam_id-1]+i,:] = color
                        for j in range(4):
                            if y[cam_id-1]+j <= 480-1 and y[cam_id-1]+j > 0:
                                image[y_0-(y[cam_id-1]+j), x_0+x[cam_id-1]+i,:] = color
                            if y[cam_id-1]-j <= 480-1 and y[cam_id-1]-j > 0:
                                image[y_0-(y[cam_id-1]-j), x_0+x[cam_id-1]+i,:] = color

                    # Horizontal Left
                    if x[cam_id-1]-i <= 640-1 and x[cam_id-1]-i > 0:
                        image[y_0-y[cam_id-1],x_0+x[cam_id-1]-i,:] = color
                        for j in range(thickness):
                            if y[cam_id-1]+j <= 480-1 and y[cam_id-1]+j > 0:
                                image[y_0-(y[cam_id-1]+j), x_0+x[cam_id-1]-i,:] = color
                            if y[cam_id-1]-j <= 480-1 and y[cam_id-1]-j > 0:
                                image[y_0-(y[cam_id-1]-j), x_0+x[cam_id-1]-i,:] = color

                    # Vertical Up
                    if y[cam_id-1]+i <= 480-1 and y[cam_id-1]+i > 0:
                        image[y_0-(y[cam_id-1]+i), x_0+x[cam_id-1],:] = color
                        for j in range(thickness):
                            if x[cam_id-1]+j <= 640-1 and x[cam_id-1]+j > 0:
                                image[y_0-(y[cam_id-1]+i),x_0+x[cam_id-1]+j,:] = color
                            if x[cam_id-1]-j <= 640-1 and x[cam_id-1]-j > 0:
                                image[y_0-(y[cam_id-1]+i),x_0+x[cam_id-1]-j,:] = color

                    # Vertical Down
                    if y[cam_id-1]-i <= 480-1 and y[cam_id-1]-i > 0:
                        image[y_0-(y[cam_id-1]-i), x_0+x[cam_id-1],:] = color
                        for j in range(thickness):
                            if x[cam_id-1]+j <= 640-1 and x[cam_id-1]+j > 0:
                                image[y_0-(y[cam_id-1]-i),x_0+x[cam_id-1]+j,:] = color
                            if x[cam_id-1]-j <= 640-1 and x[cam_id-1]-j > 0:
                                image[y_0-(y[cam_id-1]-i),x_0+x[cam_id-1]-j,:] = color



        # Horizontal Divisions
        image[0,:,:] = white
        image[480*1+1,:,:] = white
        image[480*2+2,:,:] = white

        # Vertical Divisions
        image[:,0,:] = white
        image[:,640*1+1,:] = white
        image[:,640*2+2,:] = white

        cv2.imshow("frame", image)
        cv2.waitKey(1) 

    print("Bye bye visualize")



if __name__ == "__main__":
    
    global cam_poses, r2w, r_rtl, μ, Σ, offset, focl, vis_flag


    try:
        d_source = sys.argv[1]
        if d_source != "dvs" and d_source != "opt":
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

        vis_flag = False
        vis_arg = sys.argv[3]
        if vis_arg == "on":
            print("Visualization on")
            vis_flag = True        
        else:
            vis_flag = False

    except:
        print("Try python3 merger.py <dvs|opt> <nuc|munin> <on|off>")
        quit()


    merge_queue = multiprocessing.Queue()
    target_queue = multiprocessing.Queue()
    bkgrnd_queue = multiprocessing.Queue()
    xyz_queue = multiprocessing.Queue()

    focl = set_focal_lengths()

    # Get camera poses
    cam_poses = set_cam_poses()
    
    # Get Rotation Matrices: real-cam to world 
    r2w = get_rotmats(cam_poses)
    
    # Get Translation Matrices: real-cam to world
    r_trl = get_transmats(cam_poses)


    pos_cam_1 = multiprocessing.Process(target=pos_server, args=(merge_queue,1,))
    pos_cam_2 = multiprocessing.Process(target=pos_server, args=(merge_queue,2,))
    pos_cam_3 = multiprocessing.Process(target=pos_server, args=(merge_queue,3,))

    bgi_cam_1 = multiprocessing.Process(target=img_server, args=(bkgrnd_queue, 1,))
    bgi_cam_2 = multiprocessing.Process(target=img_server, args=(bkgrnd_queue, 2,))
    bgi_cam_3 = multiprocessing.Process(target=img_server, args=(bkgrnd_queue, 3,))

    

    if d_source == "dvs" : 
        # Mean array and covariance matrix in virtual camera space
        μ = np.array([0,0,-0.75])
        Σ = np.array([[0.2,0,0],[0,0.2,0],[0,0,3.6]])    
        offset = [-0.059, 0.015, 0.034]
        merger = multiprocessing.Process(target=combiner, args=(merge_queue, target_queue, xyz_queue, ip_address, port_nb,True,))
    if d_source == "opt" :
        # Mean array and covariance matrix in virtual camera space
        μ = np.array([0,0,-0.9])
        Σ = np.array([[0.02,0,0],[0,0.02,0],[0,0,1.8]])    
        offset = [0, 0, 0]
        merger = multiprocessing.Process(target=combiner, args=(merge_queue, target_queue, xyz_queue, ip_address, port_nb,False,))

    if vis_flag:
        display = multiprocessing.Process(target=visualize, args=(target_queue, bkgrnd_queue, ))
        rt_plot = multiprocessing.Process(target=oscilloscope, args=(xyz_queue, ))

    merger.start()

    pos_cam_1.start()
    pos_cam_2.start()
    pos_cam_3.start()

    bgi_cam_1.start()
    bgi_cam_2.start()
    bgi_cam_3.start()

    if vis_flag:
        display.start()
        rt_plot.start()

    merger.join()

    if vis_flag:
        display.join()
        rt_plot.join()

    pos_cam_1.join()
    pos_cam_2.join()
    pos_cam_3.join()

    bgi_cam_1.join()
    bgi_cam_2.join()
    bgi_cam_3.join()

