
import socket
from ctypes import *
import numpy as np
import multiprocessing 

import time
import math
import datetime
import struct 
import argparse

from geometry import *

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


class RingBuffer:
    def __init__(self, size, delta):
        self.size = size
        self.buffer = [0] * size
        self.delta = delta
        self.index = 0
        self.count = 0
        self.sum = 0

    def insert(self, value):
        if self.count < self.size:
            self.count += 1
        else:
            self.sum -= self.buffer[self.index]

        self.buffer[self.index] = value
        self.sum += value
        self.index = (self.index + 1) % self.size

    def mean(self):
        if self.count == 0:
            return 0  # or raise an exception, depending on your use case
        return self.sum / self.count



##############################################################################################################################
#                                                    GAUSSIANS                                                     #
##############################################################################################################################


from scipy import stats
from scipy.stats import multivariate_normal
from scipy import stats

''' Create Multivariate Gaussian Distributions'''
def create_mgd(v2r, r2w, trl, μ, Σ, v_obj_poses):   


    r_μ = np.zeros((3,3))
    r_Σ = np.zeros((3,3,3))
    w_μ = np.zeros((3,3))
    w_Σ = np.zeros((3,3,3))
    new_μ = np.zeros((4,3)) # including a '1' at the end
    for k in range(3):
        
                                      
        # Rotating Means from virtual-cam space to real-cam space  
        r_μ[:,k] = v2r[:,:,k] @ μ[k]
                 
        # Rotating Means from real-cam space to world space 
        w_μ[:,k] = r2w[:,:,k] @ r_μ[:,k]
    
        # Translating Means from Camera (Real=Virtual) space to World space 
        new_μ[:,k] = trl[:,:, k] @ [w_μ[0,k], w_μ[1,k], w_μ[2,k],1]                     
                 
        # Rotating Covariance Matrix from virtual-cam space to real-cam space  
        r_Σ[:,:,k] = v2r[:,:,k] @ Σ[k] @ v2r[:,:,k].T  
                 
        # Rotating Covariance Matrix from real-cam space to world space  
        w_Σ[:,:,k] = r2w[:,:,k] @ r_Σ[:,:,k] @ r2w[:,:,k].T 
    
    rv_1 = multivariate_normal(new_μ[0:3,0], w_Σ[:,:,0])
    rv_2 = multivariate_normal(new_μ[0:3,1], w_Σ[:,:,1])
    rv_3 = multivariate_normal(new_μ[0:3,2], w_Σ[:,:,2])
    
    return new_μ, w_Σ, [rv_1, rv_2, rv_3]

def analytical(μ, Σ, quality, old_μ):


    mu = np.zeros(3)
    V_n_p = np.zeros((3,3)) 
    
    V_1 = np.linalg.inv(Σ[:,:,0])
    V_n_p += V_1
    μ_1 = μ[0:3,0]

    V_2 = np.linalg.inv(Σ[:,:,1])
    V_n_p += V_2
    μ_2 = μ[0:3,1]

    V_3 = np.linalg.inv(Σ[:,:,2])
    V_n_p += V_3
    μ_3 = μ[0:3,2]

    q_threshold = 0.5
    if quality[0] > q_threshold and quality[1] > q_threshold and quality[2] > q_threshold:
        V_n =np.linalg.inv(V_n_p)
        mu = ((V_1 @ μ_1) + (V_2 @ μ_2) + (V_3 @ μ_3)) @ V_n
    else:
        # Keep old estimate if quality is too low overall
        mu[0] = old_μ[0]
        mu[1] = old_μ[1]
        mu[2] = old_μ[2]

    return mu

##############################################################################################################################
#                                                    POSITION UDP SERVER                                                     #
##############################################################################################################################

def pos_server(merge_queue, cam_id, resolution):


    port_nb = 3000 + cam_id 

    off_x = int(resolution[0]/2)
    off_y = int(resolution[1]/2)


    server_addr = ('172.16.222.48', port_nb)
    ssock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ssock.bind(server_addr)
    print("Listening on port {:d}".format(port_nb))


    vis_out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print_counter = 0
    exec_counter = 0
    exec_start = time.time()
    while True:

        data, addr = ssock.recvfrom(2048)
        vis_scaler_x = 1.1
        vis_scaler_y = 1.1
        payload_in = PayloadSleipner.from_buffer_copy(data) 
        merge_queue.put([cam_id, payload_in.x, payload_in.y, payload_in.z, payload_in.a, payload_in.b, payload_in.g, payload_in.p])
        message = f"{int(payload_in.x*vis_scaler_x*off_x+off_x)},{int(payload_in.y*vis_scaler_y*off_y+off_y)}"
        vis_out_sock.sendto(message.encode(), (IP_NUC, 4330+cam_id))


        exec_counter += 1
        exec_stop = time.time()
        exec_diff = exec_stop - exec_start
        if int(exec_diff) >= 1:
            if cam_id == 1:
                print(f"Received {exec_counter} data points on cam {cam_id}")
                exec_counter = 0
                exec_start = time.time()
        

    ssock.close()


##############################################################################################################################
#                                                          COMBINER                                                          #
##############################################################################################################################

def combiner(merge_queue, ip_address, port_nb, resolution):

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
   
    ring_buff_size = 1
    delta_xyz = 0.003
    x_buff = RingBuffer(ring_buff_size, delta_xyz)
    y_buff = RingBuffer(ring_buff_size, delta_xyz)
    z_buff = RingBuffer(ring_buff_size, delta_xyz)

    # current Σ in 'virtual camera space' for cams 1|2|3
    for k in range(3):
        v_Σ[:,:,k] = Σ[k] 

    new_μ = np.zeros((4,3)) # including a '1' at the end

    px = np.zeros(3)
    py = np.zeros(3)
    pz = np.zeros(3)

    old_px = np.zeros(3)
    old_py = np.zeros(3)
    old_pz = np.zeros(3)

    quality = np.ones(3)
    prediction = np.ones(3)
    

    panda_address = ('172.16.222.48',2600)
    panda_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 


    plotter_address = ('172.16.222.46', 6000)
    plotter_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    

    counter = 0
    print_counter = 1000
    max_counter = 700
    elapsed = np.zeros(max_counter)

    exec_counter = 0
    exec_start = time.time()
    while(True):

        while not merge_queue.empty():
            datum = merge_queue.get()
            cam_id = datum[0]

            quality[cam_id-1] = datum[-1]
            
            if quality[cam_id-1] == 0:
                px[cam_id-1] = old_px[cam_id-1]
                py[cam_id-1] = old_py[cam_id-1]
                pz[cam_id-1] = old_pz[cam_id-1]

            else:
                old_px[cam_id-1] = px[cam_id-1]
                old_py[cam_id-1] = py[cam_id-1]
                old_pz[cam_id-1] = pz[cam_id-1]
                px[cam_id-1] = datum[1]*int(resolution[0]/2)
                py[cam_id-1] = datum[2]*int(resolution[1]/2)
                pz[cam_id-1] = datum[3] # @TODO: how to interpret Z? (-1 to 1?) ... what;s the scaling?
                


            r_obj_angles[:, cam_id-1] = get_angles_from_dvs(px[cam_id-1], py[cam_id-1], focl, cam_id) 
            

            start = datetime.datetime.now()

            # Estimate virtual camera poses (in real camera space)
            v_poses = set_vir_poses(r_obj_angles, v_poses)
            
            # Get Rotation Matrices: virtual-cam to real-cam 
            v2r = get_rotmats(v_poses)
            v2r = v2r[0:3,0:3,:]

            μ, Σ, offset = update_gaussians(quality)
            
            # Create Multivariate Gaussian Distributions
            new_μ, w_Σ, v_Σ = create_mgd(v2r, r2w, trl, μ, Σ, v_obj_poses)
                
            # Do predictions
            prediction = analytical(new_μ, w_Σ, quality, prediction)
            

            x_buff.insert(prediction[0]+offset[0])
            y_buff.insert(prediction[1]+offset[1])
            z_buff.insert(prediction[2]+offset[2])
            
            x = x_buff.mean()
            y = y_buff.mean()
            z = z_buff.mean()

            alpha = 0
            beta = 0
            gamma = 0


            plotter_socket.sendto(struct.pack('ffffff', x, y, z, alpha, beta, gamma), plotter_address)


            # So far we have the pose of the 'centroid' of the object in world space
            # However, the robot needs to follow one of the vertices of the object
            # The location of such vertex, the 'tip', w.r.t the centroid, is d_x, d_y, d_z)
            # We want to obtain the vertex's  location w.r.t to the origin
            # So, we need to apply translation matrices ... 


            centroid_poses = np.zeros((1,6))
            centroid_poses[0,0] = x # x
            centroid_poses[0,1] = y # y
            centroid_poses[0,2] = z # z
            centroid_poses[0,3] = alpha # alpha
            centroid_poses[0,4] = beta  # beta
            centroid_poses[0,5] = gamma # gamma


            tip_xyz = np.zeros(4)
            tip_xyz[0] = pm[18] # x: -0.050 for hammer and -0.050 for nail
            tip_xyz[1] = pm[19] # y
            tip_xyz[2] = pm[20] # z
            tip_xyz[3] = 1 # just like that
            


            centroid_r2w = get_rotmats(centroid_poses)
            centroid_trl = get_transmats(centroid_poses)
            centroid_c2w = centroid_trl[:,:,0].dot(centroid_r2w[:,:,0])

            tip_xyz = centroid_c2w.dot(tip_xyz)


            x = round(tip_xyz[0],3)
            y = round(tip_xyz[1],3)
            z = round(tip_xyz[2],3)

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
                # print(f"{x_short} | {y_short} | {z_short} | {alpha_deg} | {beta_deg} | {gamma_deg}")
                counter = 0

            
            # print(f"{x} | {y} | {z}")
            panda_socket.sendto(PayloadPanda(x, y, z, alpha, beta, gamma), panda_address)

            exec_counter += 1
            exec_stop = time.time()
            exec_diff = exec_stop - exec_start
            if int(exec_diff) >= 1:
                print(f"Counted {exec_counter} executions")
                exec_counter = 0
                exec_start = time.time()
   
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
        pm[6] = parameters['o_x_0']  # offset along X when all cameras available
        pm[7] = parameters['o_y_0']  # offset along Y when all cameras available
        pm[8] = parameters['o_z_0']  # offset along Z when all cameras available
        pm[9] =  parameters['o_x_1'] # offset along X when camera #1 is 'absent'
        pm[10] = parameters['o_y_1'] # offset along Y when camera #1 is 'absent'
        pm[11] = parameters['o_z_1'] # offset along Z when camera #1 is 'absent'
        pm[12] = parameters['o_x_2'] # offset along X when camera #2 is 'absent'
        pm[13] = parameters['o_y_2'] # offset along Y when camera #2 is 'absent'
        pm[14] = parameters['o_z_2'] # offset along Z when camera #2 is 'absent'
        pm[15] = parameters['o_x_3'] # offset along X when camera #3 is 'absent'
        pm[16] = parameters['o_y_3'] # offset along Y when camera #3 is 'absent'
        pm[17] = parameters['o_z_3'] # offset along Z when camera #3 is 'absent'
        pm[18] = parameters['tip_x'] # tip of hammer (delta_x vs centroid)
        pm[19] = parameters['tip_y'] # tip of hammer (delta_y vs centroid)
        pm[20] = parameters['tip_z'] # tip of hammer (delta_z vs centroid)
        time.sleep(1)

def update_gaussians(quality):

    μ = [] 
    Σ = []

    threshold = 0.3
    for i in range(3):


        if quality[i] >= threshold and quality[i]<=1:
            scaler = 1/quality[i]
        else:
            if quality[i] < threshold:
                scaler = 1/threshold
            elif quality[i] > 1:
                scaler = 1
                print("Quality > 1 ... how come?")

        scaler = 1
        μ.append(np.array([pm[0], pm[1], pm[2]]))
        Σ.append(np.array([[pm[3]*scaler,0,0],[0,pm[4]*scaler,0],[0,0,pm[5]*scaler]]))
    offset = [pm[6],pm[7],pm[8]]

    return μ, Σ, offset


def parse_args():

    parser = argparse.ArgumentParser(description='Merger')
    
    parser.add_argument('-x', '--res-x', type= int, help="Image scale", default=1280)
    parser.add_argument('-y', '--res-y', type= int, help="Image scale", default=720)

    return parser.parse_args()

if __name__ == "__main__":
    
    args = parse_args()

    port_nb = 2600
    ip_address = "172.16.222.48"
    resolution = [args.res_x,args.res_y]

    merge_queue = multiprocessing.Queue()

    focl = set_focal_lengths()
    cam_poses = set_cam_poses()
    r2w = get_rotmats(cam_poses)
    r2w = r2w[0:3,0:3,:]
    trl = get_transmats(cam_poses)
    


    pm = multiprocessing.Array('d', np.zeros(21))


    pos_cam_1 = multiprocessing.Process(target=pos_server, args=(merge_queue,1,resolution,))
    pos_cam_2 = multiprocessing.Process(target=pos_server, args=(merge_queue,2,resolution,))
    pos_cam_3 = multiprocessing.Process(target=pos_server, args=(merge_queue,3,resolution,))


    loader = multiprocessing.Process(target=parse_params)
    merger = multiprocessing.Process(target=combiner, args=(merge_queue, ip_address, port_nb, resolution,))

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


