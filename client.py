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
from ctypes import *

port_nb = 2300

""" This class defines a C-like struct """
class Payload(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("z", c_float)]


def main():
    server_addr = ('172.16.222.31', port_nb)
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


            panda_x = 0.0266687 + delta_x
            panda_y = -0.500253 + delta_y
            panda_z = 0.418845 + delta_z

            x = panda_x - 0.35 + 0.10
            y = panda_z - 0.01 - 0.20
            z = -panda_y + 0.36 + 0.20

            payload_out = Payload(x, y, z)
            # Sending x=-0.223331, y=0.208845, z=1.060253

            print("Sending x={:f}, y={:f}, z={:f}".format(payload_out.x, payload_out.y, payload_out.z))
            nsent = s.send(payload_out)
            # Alternative: s.sendall(...): coontinues to send data until either
            # all data has been sent or an error occurs. No return value.
            print("Sent {:d} bytes".format(nsent))
            time.sleep(0.01)

            idx += 1

    except AttributeError as ae:
        print("Error creating the socket: {}".format(ae))
    except socket.error as se:
        print("Exception on socket: {}".format(se))
    finally:
        print("Closing socket")
        s.close()


if __name__ == "__main__":
    main()
