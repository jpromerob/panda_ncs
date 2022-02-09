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


def use_dvs(queue):


    server_addr = ('172.16.222.46', 2600)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        s.connect(server_addr)
        print("Connected to {:s}".format(repr(server_addr)))

        counter = 0
        while(True):
            counter += 1
            datum = queue.get()
            cam_id = datum[0]

            px = (datum[1]+1)*320
            py = (datum[2]+1)*240
            

            payload_out = Payload(cam_id, px, py)
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



if __name__ == "__main__":
    

    queue = multiprocessing.Queue()



    cam_1 = multiprocessing.Process(target=udpserver, args=(queue,1,))
    cam_2 = multiprocessing.Process(target=udpserver, args=(queue,2,))
    cam_3 = multiprocessing.Process(target=udpserver, args=(queue,3,))

    show = multiprocessing.Process(target=use_dvs, args=(queue, ))

    show.start()
    cam_1.start()
    cam_2.start()
    cam_3.start()

    show.join()
    cam_1.join()
    cam_2.join()
    cam_3.join()




