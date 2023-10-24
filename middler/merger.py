
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
global cam_poses, r2w, rtl, trl, μ, Σ, offset, focl

""" This class defines a C-like struct """
class PayloadSleipner(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("z", c_float),
                ("p", c_float)]

class PayloadPanda(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("z", c_float)]

    
##############################################################################################################################
#                                                    POSITION UDP SERVER                                                     #
##############################################################################################################################

def pos_server(merge_queue, cam_id):


    port_nb = 3000 + cam_id%3 # cam #1 --> 3001 | cam #2 --> 3002 | cam #3 --> 3000
    server_addr = ('172.16.222.48', port_nb)
    ssock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("Socket created")

    try:
        # bind the server socket and listen
        ssock.bind(server_addr)
        ssock.listen(3)
        print("Listening on port {:d}".format(port_nb))


        vis_out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        while True:
            csock, client_address = ssock.accept()

            buff = csock.recv(512)
            while buff:
                payload_in = PayloadSleipner.from_buffer_copy(buff) 
                merge_queue.put([cam_id, payload_in.x, payload_in.y, payload_in.z, payload_in.p])
                message = f"{int(payload_in.x*320+320)},{int(payload_in.y*240+240)}"
                vis_out_sock.sendto(message.encode(), (IP_NUC, 4330+cam_id))
                
                buff = csock.recv(512)
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


##############################################################################################################################
#                                                          COMBINER                                                          #
##############################################################################################################################

def combiner(merge_queue, ip_address, port_nb):

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
    old_px = np.zeros(3)
    old_py = np.zeros(3)

    oldsence = np.ones(3)
    presence = np.ones(3)
    prediction = np.ones(3)
    olddiction = np.ones(3)
    
    server_addr = (ip_address, port_nb)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    plotter_address = ('172.16.222.46', 6000)
    plotter_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    

    try:
        s.connect(server_addr)
        print("Connected to {:s}".format(repr(server_addr)))

        counter = 0
        print_counter = 1000
        max_counter = 5000
        elapsed = np.zeros(max_counter)
        while(True):

            while not merge_queue.empty():
                datum = merge_queue.get()
                cam_id = datum[0]
                presence[cam_id-1] = datum[4]
                if oldsence[cam_id-1] != presence[cam_id-1]:
                    oldsence[cam_id-1] = presence[cam_id-1]
                
                if presence[cam_id-1] == 0:
                    px[cam_id-1] = old_px[cam_id-1]
                    py[cam_id-1] = old_py[cam_id-1]
                else:
                    old_px[cam_id-1] = px[cam_id-1]
                    old_py[cam_id-1] = py[cam_id-1]
                    px[cam_id-1] = datum[1]*320
                    py[cam_id-1] = datum[2]*240
                r_obj_angles[:, cam_id-1] = get_angles_from_dvs(px[cam_id-1], py[cam_id-1], focl, cam_id) 
                

            start = datetime.datetime.now()

            # Estimate virtual camera poses (in real camera space)
            v_poses = set_vir_poses(r_obj_angles, v_poses, presence)
            
            # Get Rotation Matrices: virtual-cam to real-cam 
            v2r = get_rotmats(v_poses)
            v2r = v2r[0:3,0:3,:]
            
            # Create Multivariate Gaussian Distributions
            new_μ, w_Σ, v_Σ = create_mgd(v2r, r2w, trl, μ, Σ, v_obj_poses)
              
            # Do predictions
            prediction = analytical(new_μ, w_Σ, presence, prediction, oldsence)
            
            x = prediction[0]+offset[0]
            y = prediction[1]+offset[1]
            z = prediction[2]+offset[2]


            stop = datetime.datetime.now()
            diff = stop - start
            elapsed[counter] = int(diff.microseconds)
            if counter < max_counter-1:
                counter += 1
            else:
                print("Elapsed time: " + str(int(np.mean(elapsed))) + " [μs].")
                counter = 0

            if counter%print_counter == 0:
                print(f"({round(x,3)},{round(y,3)},{round(z,3)})")

            # Send predicted (x,y,z) out (to robot and plotter)
            plotter_socket.sendto(struct.pack('fff', x, y, z), plotter_address)
            nsent = s.send(PayloadPanda(x, y, z))

    except AttributeError as ae:
        print("Error creating the socket: {}".format(ae))
    except socket.error as se:
        print("Exception on socket: {}".format(se))
    finally:
        print("Closing socket")
        s.close()
        plotter_socket.close()

    return 0
   


if __name__ == "__main__":
    

    port_nb = 2600
    ip_address = "172.16.222.48"

    merge_queue = multiprocessing.Queue()

    focl = set_focal_lengths()
    cam_poses = set_cam_poses()
    r2w = get_rotmats(cam_poses)
    r2w = r2w[0:3,0:3,:]
    trl = get_transmats(cam_poses)


    pos_cam_1 = multiprocessing.Process(target=pos_server, args=(merge_queue,1,))
    pos_cam_2 = multiprocessing.Process(target=pos_server, args=(merge_queue,2,))
    pos_cam_3 = multiprocessing.Process(target=pos_server, args=(merge_queue,3,))


    # Mean array and covariance matrix in virtual camera space
    μ = np.array([0,0,-0.95])
    Σ = np.array([[0.1,0,0],[0,0.1,0],[0,0,3.6]])    
    # offset = [-0.043, 0.050, 0.034]  
    offset = [-0.015,0.010,0.010]
    merger = multiprocessing.Process(target=combiner, args=(merge_queue, ip_address, port_nb,))

    merger.start()

    pos_cam_1.start()
    pos_cam_2.start()
    pos_cam_3.start()


    merger.join()

    pos_cam_1.join()
    pos_cam_2.join()
    pos_cam_3.join()


