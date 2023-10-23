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

import sensor_api


""" This class defines a C-like struct """
class Coordinates(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("z", c_float),
                ("qx", c_float),
                ("qy", c_float),
                ("qz", c_float),
                ("qw", c_float)]


class Joints(Structure):
    _fields_ = [("q0", c_double),
                ("q1", c_double),
                ("q2", c_double),
                ("q3", c_double),
                ("q4", c_double),
                ("q5", c_double),
                ("q6", c_double),
                ("x", c_double),
                ("y", c_double),
                ("z", c_double),
                ("qx", c_double),
                ("qy", c_double),
                ("qz", c_double),
                ("qw", c_double)]


def generate_coordinates():
    server_addr = ('172.16.222.31', 2600)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        s.connect(server_addr)
        print("Connected to {:s}".format(repr(server_addr)))

        print("")

        idx = 0

        start_time = time.time()

        while(True):
            

            # Measured Pose somewhere above platform
            #panda_x = 0.135259
            #panda_y = -0.466710
            #panda_z = 0.239464

            # Pos close to hand
            #panda_x = 0.107630
            #panda_y = -0.492657
            #panda_z = 0.202441

            # Pos close to hand
            amp = 0.10
            start_height = 0.10
            cur_height = (amp/2)*(math.sin(2.0*math.pi/(3.0) * (time.time() - start_time)) + 1) + start_height
            panda_x = 0.107630
            panda_y = -0.492657
            panda_z = 0.202441 + cur_height
            ( qx, qy, qz, qw ) = (1, 0, 0, 0)

            # Pos for distance keeping
            amp = 0.10
            cur_height = -(amp/2)*(math.sin(2.0*math.pi/(3.0) * (time.time() - start_time)) + 1)

            panda_x, panda_y, panda_z = (0.279682, -0.541643, 0.505104)
            qx, qy, qz, qw = (0.686426, 0.301792, 0.596582, 0.28606)
            min_x, max_x = (panda_x-0.1, panda_x)

            panda_x += cur_height

            # World Space
            x = panda_x - 0.35
            y = panda_z - 0.01
            z = -panda_y + 0.36

            xyz_out = Coordinates(x, y, z, qx, qy, qz, qw)

            #print("Sending {:f} | {:f} | {:f}".format(xyz_out.x, xyz_out.y, xyz_out.z))
            nsent = s.send(xyz_out)


            buff = s.recv(sizeof(Joints))
            if buff:
                joints_in = Joints.from_buffer_copy(buff)          

                print("Receiving {:f} | {:f} | {:f} | {:f} | {:f} | {:f} | {:f} || {:f} | {:f} | {:f} || {:f} | {:f} | {:f} | {:f}".format(
                        joints_in.q0, joints_in.q1, joints_in.q2, joints_in.q3, joints_in.q4, joints_in.q5, joints_in.q6, 
                        joints_in.x, joints_in.y, joints_in.z,
                        joints_in.qx, joints_in.qy, joints_in.qz, joints_in.qw))
                #print("Receiving {:f} | {:f} | {:f} | {:f}".format(
                #        joints_in.qx, joints_in.qy, joints_in.qz, joints_in.qw))
            time.sleep(0.001)

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
