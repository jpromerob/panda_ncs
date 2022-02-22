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
                
def udpserver(port_nb):

    global c2w

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
                print([port_nb, payload_in.x, payload_in.y, payload_in.z])
                
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







if __name__ == "__main__":
    
    global focl

    try:
        port_nb = int(sys.argv[1])
    except:
        quit()


    cam = multiprocessing.Process(target=udpserver, args=(port_nb,))
    # cam_2 = multiprocessing.Process(target=udpserver, args=(queue,2,))
    # cam_3 = multiprocessing.Process(target=udpserver, args=(queue,3,))

    cam.start()
    # cam_2.start()
    # cam_3.start()

    cam.join()
    # cam_2.join()
    # cam_3.join()




