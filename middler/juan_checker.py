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
import cv2

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


def get_dvs_from_angles(angles, focl, cam_id):

    pixels = np.zeros(2)
    
    pixels[0] = math.tan((angles[0]*math.pi/180))*focl[0,cam_id-1]
    pixels[1] = math.tan((angles[1]*math.pi/180))*focl[1,cam_id-1]

    return pixels

def use_dvs(queue,LED_queue):


    while(True):
        
        datum = queue.get()
        cam_id = datum[0]

        px = (datum[1]+1)*320
        py = (datum[2]+1)*240
        LED_queue.put([cam_id, px, py])
   
    
    return 0


def use_xyz(queue, LED_queue):

    global focl

    angles = np.zeros((2,3))
    
   
    while(True):

        datum = queue.get()
        cam_id = datum[0]

        x = np.array(datum[1])
        y = np.array(datum[2])
        z = np.array(datum[3])

        angles[0:2, cam_id-1] = get_angles_from_opt(x, y, z)

        pixels = get_dvs_from_angles(angles[0:2, cam_id-1], focl, cam_id)

        # print(" Cam#{:.0f} --> ({:.3f}, {:.3f}) ".format(cam_id, pixels[0], pixels[1]))
        LED_queue.put([cam_id, pixels[0], pixels[1]])
            


    
    return 0
       

def visualize(LED_queue):

    cam_shape = (480*2+3,640*2+3)


    x = [0, 0, 0]
    y = [0, 0, 0]

    i = 0
    print("I'm running")
    while True:

        image = np.zeros(cam_shape)


        # Horizontal Divisions
        image[0,:] = 255
        image[480*1+1,:] = 255
        image[480*2+2,:] = 255

        # Vertical Divisions
        image[:,0] = 255
        image[:,640*1+1] = 255
        image[:,640*2+2] = 255

        # All the subplots
        # image[1:480, 641:640*2+1]             Top right :     Cam#3
        # image[1:480, 1:640]                   Top left :      Cam#2
        # image[480+1:480*2+1, 1:640]           Bottom Left:    Cam#1
        # image[480+1:480*2+1, 641:640*2+1]     Empty subplot

        # Fourth (empty) subplot
        image[480+1:480*2+1, 641:640*2+1] = 255*np.ones((480,640))

        counter = 0
        while counter < 100:

            counter += 1

            datum = LED_queue.get()
            cam_id = datum[0]

            x_0 = 0
            y_0 = 0

            # Updating (x,y)
            if cam_id == 1:
                x[0] = int(datum[1])
                y[0] = int(datum[2])
            if cam_id == 2:
                x[1] = int(datum[1])
                y[1] = int(datum[2])
            if cam_id == 3:
                x[2] = int(datum[1])
                y[2] = int(datum[2])

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

            image[y_0-y[cam_id-1], x_0+x[cam_id-1]] = 255

            
            for i in range(6):
                if x[cam_id-1]+i <= 640-1:
                    image[y_0-y[cam_id-1],x_0+x[cam_id-1]+i] = 255
                if x[cam_id-1]-i > 0:
                    image[y_0-y[cam_id-1],x_0+x[cam_id-1]-i] = 255
                if y[cam_id-1]+i <= 480-1:
                    image[y_0-(y[cam_id-1]+i), x_0+x[cam_id-1]] = 255
                if y[cam_id-1]-i > 0:
                    image[y_0-(y[cam_id-1]-i), x_0+x[cam_id-1]] = 255




        cv2.imshow("frame", image)
        cv2.waitKey(1) 

    print("Bye bye visualize")



if __name__ == "__main__":
    
    global focl

    try:
        op_mode = int(sys.argv[1])
    except:
        quit()

    queue = multiprocessing.Queue()
    LED_queue = multiprocessing.Queue()

    focl = set_focal_lengths()

    cam_1 = multiprocessing.Process(target=udpserver, args=(queue,1,))
    cam_2 = multiprocessing.Process(target=udpserver, args=(queue,2,))
    cam_3 = multiprocessing.Process(target=udpserver, args=(queue,3,))
    v_all = multiprocessing.Process(target=visualize, args=(LED_queue,))

    if op_mode == 1 :
        print("Using DVS")
        show = multiprocessing.Process(target=use_dvs, args=(queue,LED_queue, ))
    if op_mode == 2 :
        print("Using OPT")
        show = multiprocessing.Process(target=use_xyz, args=(queue,LED_queue, ))

    show.start()
    v_all.start()
    cam_1.start()
    cam_2.start()
    cam_3.start()

    show.join()
    v_all.join()
    cam_1.join()
    cam_2.join()
    cam_3.join()




