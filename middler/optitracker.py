
import socket
import struct
import multiprocessing
import numpy as np
import math
import pdb
import os
from geometry import *

IP_PANDA = "172.16.222.48"
IP_NUC = "172.16.222.46"



def get_pixel_spaces_from_optitrack():

    cam_poses = set_cam_poses()
    r2w = get_rotmats(cam_poses)
    trl = get_transmats(cam_poses)
    focl = set_focal_lengths()
    pp_coor = set_pp_coordinates()

    c2w = np.zeros((4,4,3))
    for i in range(3):
        c2w[:,:,i] = trl[:,:,i].dot(r2w[:,:,i])

    opt_in_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    opt_in_socket.bind((IP_PANDA, 5000))  
    vis_out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    mrg_out_socket = []
    mrg_out_socket.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
    mrg_out_socket.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
    mrg_out_socket.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
    mrg_out_socket[0].connect((IP_PANDA, 3001) )
    mrg_out_socket[1].connect((IP_PANDA, 3002) )
    mrg_out_socket[2].connect((IP_PANDA, 3000) )

    
    while True:
        data, addr = opt_in_socket.recvfrom(1024)
        values = struct.unpack('6d', data)
        for i in range(3): # we forget the angles for now
            xyz_array[i] = round(values[i],3)
        
        ground_truth = [xyz_array[0], xyz_array[1], xyz_array[2], 1]
        perspective = get_perspectives(c2w, ground_truth)
        px_space_array = np.zeros(6)
        for i in range(3):
            angles = get_angles_from_pos(perspective[:,i])
            px_space_array[i*2], px_space_array[i*2+1] = get_dvs_from_angles(angles, focl, pp_coor, i+1)
            # px_space_array[i*2], px_space_array[i*2+1] = get_dvs_from_3dpose(perspective[:,i], focl, pp_coor, i+1)
            message = f"{int(px_space_array[i*2])},{int(px_space_array[i*2+1])}"
            vis_out_sock.sendto(message.encode(), (IP_NUC, 4331+i))
            mrg_out_socket[i].sendall(struct.pack('ffff', (px_space_array[i*2]-320)/320, -(px_space_array[i*2+1]-240)/240, 0, 1))
            

def get_optitrack_pose():
    os.system("./optitracker.exe")


if __name__ == "__main__":

    xyz_array = multiprocessing.Array('d', [0.0,0.0,0.0])
    receiver = multiprocessing.Process(target=get_pixel_spaces_from_optitrack)
    sender = multiprocessing.Process(target=get_optitrack_pose)
    receiver.start()
    sender.start()
    receiver.join()
    sender.join()