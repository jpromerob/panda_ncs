import socket
import struct
import time
import math

def send_xyz_data():

    
    IP_NUC = '172.16.222.46' 

    port_xyz_ncs = 6000
    port_xyz_opt = 5999

    x = -0.25
    y = 0.34
    z = 0.89
    

    # Create a UDP socket
    sender_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    counter = 0
    step = 2
    while(True):

        if counter+step <360:
            counter += step
        else:
            counter = 0
        
        o_x = 0.005*math.sin(counter*math.pi/180)
        o_y = 0.020*math.cos(2*counter*math.pi/180)
        o_z = 0.011*math.cos((counter+45)*math.pi/180)

        data_opt = struct.pack('fff', x, y, z)
        data_ncs = struct.pack('fff', x + o_x, y + o_y, z + o_z)
        sender_socket.sendto(data_opt, (IP_NUC, port_xyz_opt))
        sender_socket.sendto(data_ncs, (IP_NUC, port_xyz_ncs))
        time.sleep(0.050)

    sender_socket.close()

if __name__ == "__main__":


    send_xyz_data()