import aestream
import time
import cv2
import pdb
import numpy as np
import math
import argparse
import csv
import os
import multiprocessing
import socket
import matplotlib.pyplot as plt
import struct

def create_circle(radius):
    # Create a 2D array filled with zeros
    side_length = 2 * radius + 1
    circle = np.zeros((side_length, side_length))

    # Create a meshgrid of indices
    x, y = np.indices((side_length, side_length))

    # Calculate the center of the circle
    center = (radius, radius)

    # Set values in the circle to 1 (or any desired non-zero value)
    mask = (x - center[0])**2 + (y - center[1])**2 <= (radius)**2
    circle[mask] = 1

    return circle

def find_max_indices(arr):
    # Sum over axis 0
    sum_axis_0 = np.sum(arr, axis=0)
    # Sum over axis 1
    sum_axis_1 = np.sum(arr, axis=1)

    # Get indices where the maximum value occurs for each one-dimensional array
    indices_max_axis_y = np.argmax(sum_axis_0)
    indices_max_axis_x = np.argmax(sum_axis_1)

    return [indices_max_axis_x, indices_max_axis_y]

def visualize_data(args):

    rxy = [args.res_x, args.res_y]

    window_name = 'Latent Space'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, (math.ceil(rxy[0]*2*args.scale), math.ceil(rxy[1]*2*args.scale)))



    # Stream events from UDP port 3333 (default)
    mgh = 40
    mgi = 20
    mgv = 40
    radius = 10
    fth = 4 # frame thickness
    layout = np.zeros((rxy[0]*2+mgh*2+mgi,rxy[1]*2+mgv*2+mgi,3), dtype=np.uint8)

    # Top Margin (for Quadrants)
    layout[mgh-fth:mgh+rxy[0]+fth,mgv-fth:mgv,:] = 255*np.ones((rxy[0]+2*fth,fth,3), dtype=np.uint8) # Quadrant Top-Left
    layout[mgh+mgi+rxy[0]-fth:mgh+mgi+rxy[0]*2+fth,mgv-fth:mgv,:] = 255*np.ones((rxy[0]+2*fth,fth,3), dtype=np.uint8) # Quadrant Top-Right
    layout[mgh-fth:mgh+rxy[0]+fth,mgv+mgi+rxy[1]-fth:mgv+mgi+rxy[1],:] = 255*np.ones((rxy[0]+2*fth,fth,3), dtype=np.uint8) # Quadrant Bottom-Left
    layout[mgh+mgi+rxy[0]-fth:mgh+mgi+rxy[0]*2+fth,mgv+mgi+rxy[1]-fth:mgv+mgi+rxy[1],:] = 255*np.ones((rxy[0]+2*fth,fth,3), dtype=np.uint8) # Quadrant Bottom-Right

    # Bottom Margin (for Quadrants)
    layout[mgh-fth:mgh+rxy[0]+fth,mgv+rxy[1]:mgv+rxy[1]+fth,:] = 255*np.ones((rxy[0]+2*fth,fth,3), dtype=np.uint8) # Quadrant Top-Left
    layout[mgh+mgi+rxy[0]-fth:mgh+mgi+rxy[0]*2+fth,mgv+rxy[1]:mgv+rxy[1]+fth,:] = 255*np.ones((rxy[0]+2*fth,fth,3), dtype=np.uint8) # Quadrant Top-Right
    layout[mgh-fth:mgh+rxy[0]+fth,mgv+mgi+rxy[1]*2:mgv+mgi+rxy[1]*2+fth,:] = 255*np.ones((rxy[0]+2*fth,fth,3), dtype=np.uint8) # Quadrant Bottom-Left
    layout[mgh+mgi+rxy[0]-fth:mgh+mgi+rxy[0]*2+fth,mgv+mgi+rxy[1]*2:mgv+mgi+rxy[1]*2+fth,:] = 255*np.ones((rxy[0]+2*fth,fth,3), dtype=np.uint8) # Quadrant Bottom-Right

    # Left Margin (for Quadrants)
    layout[mgh-fth:mgh,mgv-fth:mgv+rxy[1]+fth,:]= 255*np.ones((fth,rxy[1]+2*fth,3), dtype=np.uint8) # Quadrant Top-Left
    layout[mgh+mgi+rxy[0]-fth:mgh+mgi+rxy[0],mgv-fth:mgv+rxy[1]+fth,:]= 255*np.ones((fth,rxy[1]+2*fth,3), dtype=np.uint8) # Quadrant Top-Right
    layout[mgh-fth:mgh,mgv+mgi+rxy[1]-fth:mgv+mgi+rxy[1]*2+fth,:]= 255*np.ones((fth,rxy[1]+2*fth,3), dtype=np.uint8) # Quadrant Bottom-Left
    layout[mgh+mgi+rxy[0]-fth:mgh+mgi+rxy[0],mgv+mgi+rxy[1]-fth:mgv+mgi+rxy[1]*2+fth,:]= 255*np.ones((fth,rxy[1]+2*fth,3), dtype=np.uint8) # Quadrant Bottom-Right

    # Right Margin (for Quadrants)
    layout[mgh+rxy[0]:mgh+rxy[0]+fth,mgv-fth:mgv+rxy[1]+fth,:]= 255*np.ones((fth,rxy[1]+2*fth,3), dtype=np.uint8) # Quadrant Top-Left
    layout[mgh+mgi+rxy[0]*2:mgh+mgi+rxy[0]*2+fth,mgv-fth:mgv+rxy[1]+fth,:]= 255*np.ones((fth,rxy[1]+2*fth,3), dtype=np.uint8) # Quadrant Top-Right
    layout[mgh+rxy[0]:mgh+rxy[0]+fth,mgv+mgi+rxy[1]-fth:mgv+mgi+rxy[1]*2+fth,:]= 255*np.ones((fth,rxy[1]+2*fth,3), dtype=np.uint8) # Quadrant Bottom-Left
    layout[mgh+mgi+rxy[0]*2:mgh+mgi+rxy[0]*2+fth,mgv+mgi+rxy[1]-fth:mgv+mgi+rxy[1]*2+fth,:]= 255*np.ones((fth,rxy[1]+2*fth,3), dtype=np.uint8) # Quadrant Bottom-Right

    frame = layout
    marker = np.zeros((2*radius+1, 2*radius+1, 3), dtype=np.uint8)

    xtra = 5
    top_r_1 = (xtra,mgv)
    bot_r_1 = (mgh-2*xtra, mgv+rxy[1])
    top_r_2 = (xtra,mgv+mgi+rxy[1])
    bot_r_2 = (mgh-2*xtra, mgv+mgi+2*rxy[1])
    top_r_3 = (mgh+mgi+2*rxy[0]+2*xtra,mgv)
    bot_r_3 = (mgh*2-2*xtra+mgi+2*rxy[0]+xtra, mgv+rxy[1])
    


    # ncs_logo = cv2.imread(f'ncs_logo_{rxy[0]}x{rxy[1]}.png') 
    red = (0, 0, 255) 
    orange = (0, 128, 255) 
    ring_th = 5

    mask_radius = 5
    mask = create_circle(mask_radius)
    print(mask.shape)
    # plt.imshow(mask, cmap='gray', interpolation='nearest')
    # plt.show()


    IP_PANDA = "172.16.222.48"
    mrg_out_socket = []
    mrg_address = []

    mrg_out_socket.append(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
    mrg_out_socket.append(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
    mrg_out_socket.append(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))

    mrg_address.append((IP_PANDA, 3001))
    mrg_address.append((IP_PANDA, 3002))
    mrg_address.append((IP_PANDA, 3003))


    new_px_x = [int(args.res_x/2),int(args.res_x/2),int(args.res_x/2)]
    old_px_x = [int(args.res_x/2),int(args.res_x/2),int(args.res_x/2)]

    new_px_y = [int(args.res_y/2),int(args.res_y/2),int(args.res_y/2)]
    old_px_y = [int(args.res_y/2),int(args.res_y/2),int(args.res_y/2)]

    while(True):
        # Reshape the array to its original shape (640x480)

        latent_all = []
        masked_all = []

        masked_all.append(np.zeros((args.res_x, args.res_y)))
        masked_all.append(np.zeros((args.res_x, args.res_y)))
        masked_all.append(np.zeros((args.res_x, args.res_y)))

        latent_all.append(np.frombuffer(cam_array_1.get_obj(), dtype=np.float64).reshape((args.res_x, args.res_y)))
        latent_all.append(np.frombuffer(cam_array_2.get_obj(), dtype=np.float64).reshape((args.res_x, args.res_y)))
        latent_all.append(np.frombuffer(cam_array_3.get_obj(), dtype=np.float64).reshape((args.res_x, args.res_y)))



        for i in range(3):
            xy = [old_px_x[i], old_px_y[i]]
            if xy[0]>mask_radius and xy[1]>mask_radius and xy[0] < args.res_x-mask_radius and xy[1] < args.res_y-mask_radius:
                masked_all[i][xy[0]-mask_radius:xy[0]+mask_radius+1,xy[1]-mask_radius:xy[1]+mask_radius+1] = mask
                latent_all[i] = latent_all[i] * (params[i]) # scaling the latent spaces (so they are visible)
                # latent_all[i] = (latent_all[i] > params[3]).astype(int) # thresholding latent spaces (to remove noise)
                latent_all[i] = latent_all[i]*masked_all[i] # masking latent spaces (to focus around center)
                
                new_xy = find_max_indices(latent_all[i])
                if new_xy[0]>mask_radius and new_xy[1]>mask_radius and new_xy[0] < args.res_x-mask_radius and new_xy[1] < args.res_y-mask_radius:
                    px_x = (new_xy[0]-args.res_x/2)*2/args.res_x
                    px_y = -(new_xy[1]-args.res_y/2)*2/args.res_y
                    data_cam = struct.pack('fffffff', px_x, px_y, 0, 0, 0, 0, 1)
                    mrg_out_socket[i].sendto(data_cam, mrg_address[i])
                    old_px_x[i] =  new_xy[0]
                    old_px_y[i] =  new_xy[1]

                

        frame[mgh+0:mgh+rxy[0],mgv+mgi+rxy[1]:mgv+mgi+rxy[1]*2,0:2] = 255*latent_all[0][:,:,np.newaxis] # cyan
        frame[mgh+0:mgh+rxy[0],mgv+0:mgv+rxy[1],1] =  255*latent_all[1] # green
        frame[mgh+mgi+rxy[0]:mgh+mgi+rxy[0]*2,mgv+0:mgv+rxy[1],1:3] = 255*latent_all[2][:,:,np.newaxis] # yellow

        image = cv2.resize(frame.transpose(1,0,2), (math.ceil((rxy[0]*2+mgh*2+mgi)*args.scale),math.ceil((rxy[1]*2+mgv*2+mgi)*args.scale)), interpolation = cv2.INTER_AREA)
        

        cv2.imshow(window_name, image)
        cv2.waitKey(1)


def receive_tensor(args, cam_id):
    
    # Set up TCP server
    TCP_IP = "172.16.222.46"  # Replace with the server's IP address
    port = args.port + cam_id 

    while(True):

        # Create a TCP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind((TCP_IP, port))
        sock.listen(1)

        print(f"Waiting for a connection on port {port}...")

        # Accept a connection from a client
        conn, addr = sock.accept()
        print(f"Connection accepted on port {port} (address: {addr}")

        # Determine the size of the array
        array_size = args.res_x * args.res_y * 8  # Assuming float64 data type


        while(True):

            try:
                # Receive the array data
                array_data = b""
                while len(array_data) < array_size:
                    data_chunk = conn.recv(array_size - len(array_data))
                    if not data_chunk:
                        break
                    array_data += data_chunk

                # Convert received bytes to NumPy array
                array_data = np.frombuffer(array_data, dtype=np.float64)  # Adjust dtype accordingly
                if cam_id == 1:
                    np.copyto(np.frombuffer(cam_array_1.get_obj(), dtype=np.float64), array_data)
                if cam_id == 2:
                    np.copyto(np.frombuffer(cam_array_2.get_obj(), dtype=np.float64), array_data)
                if cam_id == 3:
                    np.copyto(np.frombuffer(cam_array_3.get_obj(), dtype=np.float64), array_data)
            except:
                print(f"Closing socket for Cam #{cam_id}")
                conn.close()
                sock.close()
                break
        time.sleep(1)

def update_parameters():

    while(True):
        data = []
        file_path = "scales.txt"
        try:
            with open(file_path, 'r') as file:
                for line in file:
                    # Split each line based on ':' and extract the second part as a float
                    _, value_str = line.split(':')
                    value = float(value_str.strip())
                    data.append(value)

        except FileNotFoundError:
            print(f"Error: File '{file_path}' not found.")
        except Exception as e:
            print(f"Error: {e}")



        for i in range(4):
            if params[i] != data[i]:
                if i<3:
                    print(f"New scale Cam {i+1}: {data[i]}")
                else:
                    print(f"New Threshold: {data[i]}")
                params[i] = data[i]


        time.sleep(1)


def parse_args():

    parser = argparse.ArgumentParser(description='Cobotics Workspace Visualizer and Streamer')
        
    parser.add_argument('-p', '--port', type= int, help="Port for events Cam #1", default=4000)

    parser.add_argument('-s', '--scale', type= float, help="Image scale", default=1)

    parser.add_argument('-x', '--res-x', type= int, help="Cam Width", default=119)
    parser.add_argument('-y', '--res-y', type= int, help="Cam Height", default=72)

    return parser.parse_args()
        

if __name__ == '__main__':

    args = parse_args()

    params = multiprocessing.Array('d', [0.0,0.0,0.0,0.0])
    

    full_res = int(args.res_x*args.res_y)

    cam_array_1 = multiprocessing.Array('d', np.zeros(int(args.res_x*args.res_y), dtype=np.float64))
    cam_array_2 = multiprocessing.Array('d', np.zeros(int(args.res_x*args.res_y), dtype=np.float64))
    cam_array_3 = multiprocessing.Array('d', np.zeros(int(args.res_x*args.res_y), dtype=np.float64))
   
    # # Create three parallel processes
    p_receiver_1 = multiprocessing.Process(target=receive_tensor, args=(args, 1,))
    p_receiver_2 = multiprocessing.Process(target=receive_tensor, args=(args, 2,))
    p_receiver_3 = multiprocessing.Process(target=receive_tensor, args=(args, 3,))
    p_visualization = multiprocessing.Process(target=visualize_data, args=(args,))
    p_parametrizer = multiprocessing.Process(target=update_parameters)

    # Start the processes
    p_visualization.start()
    p_receiver_1.start()
    p_receiver_2.start()
    p_receiver_3.start()
    p_parametrizer.start()

    # Wait for the processes to finish (you may need to manually stop them)
    p_visualization.join()
    p_receiver_1.join()
    p_receiver_2.join()
    p_receiver_3.join()
    p_parametrizer.join()