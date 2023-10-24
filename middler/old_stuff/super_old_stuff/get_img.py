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
import cv2


##############################################################################################################################
#                                                         UDP SERVER                                                         #
##############################################################################################################################

def udpserver(cam_id):


    port_nb = 4000 + cam_id%3 # cam #1 --> 3001 | cam #2 --> 3002 | cam #3 --> 3000
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

            buff = csock.recv((2+480)*4)
            while buff:

                # payload_in = PayloadSleipner.from_buffer_copy(buff)       
                # presence = np.random.randint(2)
                # merge_queue.put([cam_id, payload_in.x, payload_in.y, payload_in.z, payload_in.p])
                
                buff = csock.recv((2+480)*4)
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
    


    cam_1 = multiprocessing.Process(target=udpserver, args=(1,))
    cam_2 = multiprocessing.Process(target=udpserver, args=(2,))
    cam_3 = multiprocessing.Process(target=udpserver, args=(3,))

    cam_1.start()
    # cam_2.start()
    # cam_3.start()

    cam_1.join()
    # cam_2.join()
    # cam_3.join()

