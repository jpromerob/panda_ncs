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

        panda_x = 0.400
        panda_y = 0.000 
        panda_z = 0.1

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




    except AttributeError as ae:
        print("Error creating the socket: {}".format(ae))
    except socket.error as se:
        print("Exception on socket: {}".format(se))
    finally:
        print("Closing socket")
        s.close()


if __name__ == "__main__":
    main()
