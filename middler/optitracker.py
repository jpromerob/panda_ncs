
import socket
import struct
import multiprocessing
import numpy as np
import math
import pdb
import time
import os
from geometry import *
import argparse

IP_PANDA = "172.16.222.48"
IP_NUC = "172.16.222.46"


def presentiatior():

    max_small_counter = 400
    shall_print = False

    combinations = []

    for i in [1,0]:
        for j in [1, 0]:
            for k in [1, 0]:
                combinations.append([i, j, k])

    small_counter = -1
    large_counter = 0
    while(True):
        small_counter += 1
        if small_counter >= max_small_counter:
            shall_print=True
            large_counter+=1
            small_counter = -1

        output = combinations[large_counter%8]
        if shall_print:
            shall_print = False
        if sum(output)>=2:
            yield output


def get_pixel_spaces_from_optitrack(disable_opt_px):

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
    
    mrg_out_socket = [] # mrg as in MeRGer
    mrg_address_euler = []
    mrg_address_quatr = []
    # mrg_out_socket.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
    # mrg_out_socket.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
    # mrg_out_socket.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
    # mrg_out_socket[0].connect((IP_PANDA, 3001) )
    # mrg_out_socket[1].connect((IP_PANDA, 3002) )
    # mrg_out_socket[2].connect((IP_PANDA, 3000) )

    mrg_out_socket.append(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
    mrg_out_socket.append(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
    mrg_out_socket.append(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))

    mrg_address_euler.append((IP_PANDA, 3001))
    mrg_address_euler.append((IP_PANDA, 3002))
    mrg_address_euler.append((IP_PANDA, 3003))

    mrg_address_quatr.append((IP_PANDA, 3601))
    mrg_address_quatr.append((IP_PANDA, 3602))
    mrg_address_quatr.append((IP_PANDA, 3603))

    
    plotter_address = ('172.16.222.46', 5999)
    plotter_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


    gen_presence = presentiatior()
    trash_counter = 0
    while True:
        trash_counter+=1
        data, addr = opt_in_socket.recvfrom(1024)
        values = struct.unpack('10d', data)
        q_array[0] = round(values[6],3)
        for i in range(3): # we forget the angles for now
            xyz_array[i] = round(values[i],3)
            abg_array[i] = round(values[i+3],3)
            q_array[i+1] = round(values[i+7],3)

        if trash_counter % 100 == 0:
            alpha_deg = round(abg_array[0],2)
            beta_deg =  round(abg_array[1],2)
            gamma_deg = round(abg_array[2],2)
            print(f"{alpha_deg} | {beta_deg} | {gamma_deg}")
        
        xyz_ground_truth = [xyz_array[0], xyz_array[1], xyz_array[2], 1]
        plotter_socket.sendto(struct.pack('ffffff', xyz_array[0], xyz_array[1], xyz_array[2], abg_array[0], abg_array[1], abg_array[2]), plotter_address)

        if not disable_opt_px:
            perspective = get_perspectives(c2w, xyz_ground_truth)
            px_space_array = np.zeros(6)
            presence = [1,1,1] #next(gen_presence)
            for i in range(3): # for each camera
                angles = get_angles_from_pos(perspective[:,i])
                px_space_array[i*2], px_space_array[i*2+1] = get_dvs_from_angles(angles, focl, pp_coor, i+1)
                                
                px_x = (px_space_array[i*2]-320)/320
                px_y = (px_space_array[i*2+1]-240)/240
                px_z = 0

                # To calculate the orientation (roll, pitch, yaw) 
                # of the object from the perspective of the camera, 
                # you can subtract the camera's orientation from the 
                # object's orientation in the 'world' frame. 

                px_a = (abg_array[0]*math.pi/180 - cam_poses[i][3])/math.pi # from -1 to 1 (instead of -pi to pi)
                px_b = (abg_array[1]*math.pi/180 - cam_poses[i][4])/math.pi # from -1 to 1 (instead of -pi to pi)
                px_g = (abg_array[2]*math.pi/180 - cam_poses[i][5])/math.pi # from -1 to 1 (instead of -pi to pi)

                px_qw = q_array[0]
                px_qx = q_array[1]
                px_qy = q_array[2]
                px_qz = q_array[3]
                # if trash_counter % 100 == 0:
                #     alpha_deg = round(px_a*180/math.pi,2)
                #     beta_deg =  round(px_b*180/math.pi,2)
                #     gamma_deg = round(px_g*180/math.pi,2)
                #     print(f"cam #{i+1} {alpha_deg} | {beta_deg} | {gamma_deg}")

                data_euler = struct.pack('fffffff', px_x, px_y, px_z, px_a, px_b, px_g, presence[i])
                data_quatr = struct.pack('ffffffff', px_x, px_y, px_z, px_qw, px_qx, px_qy, px_qz, presence[i])
                
                mrg_out_socket[i].sendto(data_euler, mrg_address_euler[i])
                mrg_out_socket[i].sendto(data_quatr, mrg_address_quatr[i])
    
    plotter_socket.close()        

def get_optitrack_pose():
    os.system("./new_optitracker.exe")

def parse_args():

    parser = argparse.ArgumentParser(description='Optitracker')
    
    parser.add_argument('-d', '--disable-opt-px', action='store_true', help="Disable Optitrack's pixel space")

    return parser.parse_args()
        

if __name__ == '__main__':


    args = parse_args()


    xyz_array = multiprocessing.Array('d', [0.0,0.0,0.0])
    abg_array = multiprocessing.Array('d', [0.0,0.0,0.0])
    q_array = multiprocessing.Array('d', [0.0,0.0,0.0,0.0])
    receiver = multiprocessing.Process(target=get_pixel_spaces_from_optitrack, args=(args.disable_opt_px,))
    sender = multiprocessing.Process(target=get_optitrack_pose)
    receiver.start()
    sender.start()
    receiver.join()
    sender.join()