
import socket
from ctypes import *
import numpy as np
import multiprocessing 

import time
import math
import datetime
import struct 

from geometry import *
from gaussians import *

IP_NUC = "172.16.222.46"
global cam_poses, focl, r2w, rtl, trl

""" This class defines a C-like struct """
class PayloadSleipner(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("z", c_float),
                ("a", c_float),
                ("b", c_float),
                ("g", c_float),
                ("p", c_float)]

class PayloadPanda(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("z", c_float),
                ("a", c_float),
                ("b", c_float),
                ("g", c_float)]

    
##############################################################################################################################
#                                                    POSITION UDP SERVER                                                     #
##############################################################################################################################

def pos_server(merge_queue, cam_id):


    port_nb = 3000 + cam_id 



    server_addr = ('172.16.222.48', port_nb)
    ssock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ssock.bind(server_addr)
    print("Listening on port {:d}".format(port_nb))


    vis_out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:

        data, addr = ssock.recvfrom(2048)

        payload_in = PayloadSleipner.from_buffer_copy(data) 
        merge_queue.put([cam_id, payload_in.x, payload_in.y, payload_in.z, payload_in.a, payload_in.b, payload_in.g, payload_in.p])
        message = f"{int(payload_in.x*320+320)},{int(payload_in.y*240+240)}"
        vis_out_sock.sendto(message.encode(), (IP_NUC, 4330+cam_id))
        

    ssock.close()


##############################################################################################################################
#                                                          COMBINER                                                          #
##############################################################################################################################

def combiner(merge_queue, ip_address, port_nb):

    time.sleep(1)
    μ, Σ, offset = update_gaussians([1,1,1])

    v_poses = np.zeros((3,6))

    r_obj_poses = np.zeros((3,3))       
    r_obj_angles = np.zeros((2,3))   
    v_obj_poses = np.zeros((3,3))  
    v_obj_angles = np.zeros((2,3))   

    r_μ = np.zeros((3,3))
    r_Σ = np.zeros((3,3,3))
    w_μ = np.zeros((3,3))
    w_Σ = np.zeros((3,3,3))

    v_Σ = np.zeros((3,3,3))
   

    # current Σ in 'virtual camera space' for cams 1|2|3
    for k in range(3):
        v_Σ[:,:,k] = Σ 

    new_μ = np.zeros((4,3)) # including a '1' at the end

    px = np.zeros(3)
    py = np.zeros(3)
    pz = np.zeros(3)

    pa = np.zeros(3)
    pb = np.zeros(3)
    pg = np.zeros(3)

    old_px = np.zeros(3)
    old_py = np.zeros(3)
    old_pz = np.zeros(3)

    old_pa = np.zeros(3)
    old_pb = np.zeros(3)
    old_pg = np.zeros(3)

    oldsence = np.ones(3)
    presence = np.ones(3)
    prediction = np.ones(3)
    olddiction = np.ones(3)
    

    panda_address = ('172.16.222.48',2600)
    panda_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 


    plotter_address = ('172.16.222.46', 6000)
    plotter_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    

    counter = 0
    print_counter = 1000
    max_counter = 1400
    elapsed = np.zeros(max_counter)

    while(True):

        while not merge_queue.empty():
            datum = merge_queue.get()
            cam_id = datum[0]
            presence[cam_id-1] = datum[-1]
            if oldsence[cam_id-1] != presence[cam_id-1]:
                oldsence[cam_id-1] = presence[cam_id-1]
            
            if presence[cam_id-1] == 0:
                px[cam_id-1] = old_px[cam_id-1]
                py[cam_id-1] = old_py[cam_id-1]
                pz[cam_id-1] = old_pz[cam_id-1]
                pa[cam_id-1] = old_pa[cam_id-1]
                pb[cam_id-1] = old_pb[cam_id-1]
                pg[cam_id-1] = old_pg[cam_id-1]

            else:
                old_px[cam_id-1] = px[cam_id-1]
                old_py[cam_id-1] = py[cam_id-1]
                old_pz[cam_id-1] = pz[cam_id-1]
                old_pa[cam_id-1] = pa[cam_id-1]
                old_pb[cam_id-1] = pb[cam_id-1]
                old_pg[cam_id-1] = pg[cam_id-1]
                px[cam_id-1] = datum[1]*320
                py[cam_id-1] = datum[2]*240
                pz[cam_id-1] = datum[3] # @TODO: how to interpret Z? (-1 to 1?) ... what;s the scaling?
                pa[cam_id-1] = datum[4]*math.pi # @TODO: assuming that angles in [-1..1] (from -pi to pi)
                pb[cam_id-1] = datum[5]*math.pi
                pg[cam_id-1] = datum[6]*math.pi


            r_obj_angles[:, cam_id-1] = get_angles_from_dvs(px[cam_id-1], py[cam_id-1], focl, cam_id) 
            

        start = datetime.datetime.now()

        # Estimate virtual camera poses (in real camera space)
        v_poses = set_vir_poses(r_obj_angles, v_poses, presence)
        
        # Get Rotation Matrices: virtual-cam to real-cam 
        v2r = get_rotmats(v_poses)
        v2r = v2r[0:3,0:3,:]

        μ, Σ, offset = update_gaussians(presence)
        
        # Create Multivariate Gaussian Distributions
        new_μ, w_Σ, v_Σ = create_mgd(v2r, r2w, trl, μ, Σ, v_obj_poses)
            
        # Do predictions
        prediction = analytical(new_μ, w_Σ, presence, prediction, oldsence)
        
        x = prediction[0]+offset[0]
        y = prediction[1]+offset[1]
        z = prediction[2]+offset[2]
        
        alpha, beta, gamma = average_angles(pa, pb, pg, presence, cam_poses)



        plotter_socket.sendto(struct.pack('ffffff', x, y, z, alpha, beta, gamma), plotter_address)


        # So far we have the pose of the 'centroid' of the object in world space
        # However, the robot needs to follow one of the vertices of the object
        # The location of such vertex, w.r.t the centroid, is d_x, d_y, d_z)
        # We want to obtain the vertex's  location w.r.t to the origin
        # So, we need to apply translation matrices ... 


        centroid_poses = np.zeros((1,6))
        centroid_poses[0,0] = x # x
        centroid_poses[0,1] = y # y
        centroid_poses[0,2] = z # z
        centroid_poses[0,3] = alpha # alpha
        centroid_poses[0,4] = beta  # beta
        centroid_poses[0,5] = gamma # gamma


        vertex_xyz = np.zeros(4)
        vertex_xyz[0] = -0.050 # x: -0.050 for hammer and -0.050 for nail
        vertex_xyz[1] = -0.000 # y
        vertex_xyz[2] =  0.135 # z
        vertex_xyz[3] = 1 # just like that

        vertex_abg = np.zeros(4)
        vertex_abg[0] = (math.pi/180)*-0
        vertex_abg[1] = (math.pi/180)*90
        vertex_abg[2] = (math.pi/180)*-0
        


        centroid_r2w = get_rotmats(centroid_poses)
        centroid_trl = get_transmats(centroid_poses)
        centroid_c2w = centroid_trl[:,:,0].dot(centroid_r2w[:,:,0])
        vertex_xyz = centroid_c2w.dot(vertex_xyz)

        x = round(vertex_xyz[0],3)
        y = round(vertex_xyz[1],3)
        z = round(vertex_xyz[2],3)

        theta = math.atan2(abs(vertex_xyz[0]-centroid_poses[0,0]),abs(vertex_xyz[2]-centroid_poses[0,2]))
        phi = math.atan2(abs(vertex_xyz[1]-centroid_poses[0,1]),abs(vertex_xyz[2]-centroid_poses[0,2]))
        eta = math.atan2(abs(vertex_xyz[1]-centroid_poses[0,1]),abs(vertex_xyz[0]-centroid_poses[0,0]))

        stop = datetime.datetime.now()
        diff = stop - start
        elapsed[counter] = int(diff.microseconds)
        if counter < max_counter-1:
            counter += 1
        else:
            print("Elapsed time: " + str(int(np.mean(elapsed))) + " [μs].")
            x_short = round(x,3)
            y_short = round(y,3)
            z_short = round(z,3)
            alpha_deg = round(alpha*180/math.pi,2)
            beta_deg =   round(beta*180/math.pi,2)
            gamma_deg = round(gamma*180/math.pi,2)
            print(f"{x_short} | {y_short} | {z_short} | {alpha_deg} | {beta_deg} | {gamma_deg}")
            counter = 0

        
        panda_socket.sendto(PayloadPanda(x, y, z, alpha, beta, gamma), panda_address)

   
    s.close()
    plotter_socket.close()

    return 0


def parse_paramerge_cfg():


    file_path = 'paramerge.cfg'

    # Define a dictionary to store the parameter values
    parameters = {}

    # Open and read the config file
    with open(file_path, 'r') as file:
        for line in file:
            key, value = line.strip().split(': ')
            parameters[key] = float(value)

    # Extract values for the parameters
    return parameters



def parse_params():
    
    global μ, Σ, offset

    while(True):
        parameters = parse_paramerge_cfg()
        pm[0] = parameters['μ_x']
        pm[1] = parameters['μ_y']
        pm[2] = parameters['μ_z']
        pm[3] = parameters['Σ_x']
        pm[4] = parameters['Σ_y']
        pm[5] = parameters['Σ_z']
        pm[6] = parameters['o_x_0']
        pm[7] = parameters['o_y_0']
        pm[8] = parameters['o_z_0']
        pm[9] =  parameters['o_x_1']
        pm[10] = parameters['o_y_1']
        pm[11] = parameters['o_z_1']
        pm[12] = parameters['o_x_2']
        pm[13] = parameters['o_y_2']
        pm[14] = parameters['o_z_2']
        pm[15] = parameters['o_x_3']
        pm[16] = parameters['o_y_3']
        pm[17] = parameters['o_z_3']
        time.sleep(1)

def update_gaussians(presence):

    μ = np.array([pm[0], pm[1], pm[2]])
    Σ = np.array([[pm[3],0,0],[0,pm[4],0],[0,0,pm[5]]])
    if sum(presence) == 3:
        offset = [pm[6],pm[7],pm[8]]
    else:
        if sum(presence)==2:
            if presence[0]==0:
                offset = [pm[9],pm[10],pm[11]]
            elif presence[1]==0:
                offset = [pm[12],pm[13],pm[14]]
            elif presence[2]==0:
                offset = [pm[15],pm[16],pm[17]]
        else:
            offset = [0,0,0]

    return μ, Σ, offset



if __name__ == "__main__":
    

    port_nb = 2600
    ip_address = "172.16.222.48"

    merge_queue = multiprocessing.Queue()

    focl = set_focal_lengths()
    cam_poses = set_cam_poses()
    r2w = get_rotmats(cam_poses)
    r2w = r2w[0:3,0:3,:]
    trl = get_transmats(cam_poses)
    


    pm = multiprocessing.Array('d', [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])


    pos_cam_1 = multiprocessing.Process(target=pos_server, args=(merge_queue,1,))
    pos_cam_2 = multiprocessing.Process(target=pos_server, args=(merge_queue,2,))
    pos_cam_3 = multiprocessing.Process(target=pos_server, args=(merge_queue,3,))


    loader = multiprocessing.Process(target=parse_params)
    merger = multiprocessing.Process(target=combiner, args=(merge_queue, ip_address, port_nb,))

    loader.start()
    merger.start()

    pos_cam_1.start()
    pos_cam_2.start()
    pos_cam_3.start()

    pos_cam_1.join()
    pos_cam_2.join()
    pos_cam_3.join()

    merger.join()
    loader.join()


