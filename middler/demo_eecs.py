#!/usr/bin/env python3

""" client.py - Echo client for sending/receiving C-like structs via socket
References:
- Ctypes: https://docs.python.org/3/library/ctypes.html
- Sockets: https://docs.python.org/3/library/socket.html
"""

import socket
import sys
import math
import random
import time
import multiprocessing
from ctypes import *


""" This class defines a C-like struct """
class Coordinates(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("z", c_float)]


class Joints(Structure):
    _fields_ = [("q0", c_double),
                ("q1", c_double),
                ("q2", c_double),
                ("q3", c_double),
                ("q4", c_double),
                ("q5", c_double),
                ("q6", c_double)]


def generate_coordinates():
    server_addr = ('172.16.222.31', 2600)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        s.connect(server_addr)
        print("Connected to {:s}".format(repr(server_addr)))

        print("")

        idx = 0
        max_radius = 0.10
        while(True):
            

            # Create a trajectory whose projections on axes x, y, and z are circles
            delta_x = max_radius*math.sin(math.pi/180*0.5*idx)
            delta_y = max_radius*math.sin(math.pi/180*2*idx)
            delta_z = max_radius*math.sin(math.pi/180*idx)


            panda_x = 0.000 + delta_x
            panda_y = -0.600 + delta_y 
            panda_z = 0.300 + delta_z

            x = panda_x - 0.35
            y = panda_z - 0.01
            z = -panda_y + 0.36

            xyz_out = Coordinates(x, y, z)

            print("Sending {:f} | {:f} | {:f}".format(xyz_out.x, xyz_out.y, xyz_out.z))
            nsent = s.send(xyz_out)


            # buff = s.recv(sizeof(Joints))
            # if buff:
            #     joints_in = Joints.from_buffer_copy(buff)          

            #     print("Receiving {:f} | {:f} | {:f} | {:f} | {:f} | {:f} | {:f}".format(joints_in.q0, joints_in.q1, joints_in.q2, joints_in.q3, joints_in.q4, joints_in.q5, joints_in.q6))
            time.sleep(0.05)

            idx += 2

    except AttributeError as ae:
        print("Error creating the socket: {}".format(ae))
    except socket.error as se:
        print("Exception on socket: {}".format(se))
    finally:
        print("Closing socket")
        s.close()




if __name__ == "__main__":
    
    coor_process = multiprocessing.Process(target=generate_coordinates, args=())

    coor_process.start()   
    coor_process.join()