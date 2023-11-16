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

def visualize_data(args):

    rxy = [args.res_x, args.res_y]

    window_name = 'Pixel Space'
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


    # ncs_logo = cv2.imread(f'ncs_logo_{rxy[0]}x{rxy[1]}.png') 



    
    # Set up TCP server
    TCP_IP = "172.16.222.46"  # Replace with the server's IP address
    TCP_PORT = 4001  # Replace with the server's port number

    RES_X = 119
    RES_Y = 72


    # Create a TCP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((TCP_IP, TCP_PORT))
    sock.listen(1)

    print("Waiting for a connection...")

    # Accept a connection from a client
    conn, addr = sock.accept()
    print("Connection address:", addr)

    # Determine the size of the array
    array_size = RES_X * RES_Y * 8  # Assuming float64 data type



    red = (0, 0, 255) 
    orange = (0, 128, 255) 
    ring_th = 5

    while(True):
        # Receive the array data
        array_data_1 = b""
        while len(array_data_1) < array_size:
            data_chunk = conn.recv(array_size - len(array_data_1))
            if not data_chunk:
                break
            array_data_1 += data_chunk

        # Convert received bytes to NumPy array
        array_data_1 = np.frombuffer(array_data_1, dtype=np.float64)  # Adjust dtype accordingly

        # Reshape the array to its original shape (640x480)
        array_data_1 = array_data_1.reshape((RES_X, RES_Y))
        maxi = np.max(array_data_1)
        array_data_1 = array_data_1 * (255/maxi)

        # print(array_data_1)
        print(maxi)
        print(f"max : {maxi} min {np.min(array_data_1)}")

        


                

        frame[mgh+0:mgh+rxy[0],mgv+mgi+rxy[1]:mgv+mgi+rxy[1]*2,0:2] = 255*array_data_1[:,:,np.newaxis] # cyan
        # frame[mgh+0:mgh+rxy[0],mgv+0:mgv+rxy[1],1] =  255*array_2 # green
        # frame[mgh+mgi+rxy[0]:mgh+mgi+rxy[0]*2,mgv+0:mgv+rxy[1],1:3] = 255*array_3[:,:,np.newaxis] # yellow

        image = cv2.resize(frame.transpose(1,0,2), (math.ceil((rxy[0]*2+mgh*2+mgi)*args.scale),math.ceil((rxy[1]*2+mgv*2+mgi)*args.scale)), interpolation = cv2.INTER_AREA)
        
    
        cv2.imshow(window_name, image)
        cv2.waitKey(1)

    # Close the connection and socket
    conn.close()
    sock.close()

def parse_args():

    parser = argparse.ArgumentParser(description='Cobotics Workspace Visualizer and Streamer')
        
    parser.add_argument('-p1', '--port1', type= int, help="Port for events Cam #1", default=4201)
    parser.add_argument('-p2', '--port2', type= int, help="Port for events Cam #2", default=4202)
    parser.add_argument('-p3', '--port3', type= int, help="Port for events Cam #3", default=4203)

    parser.add_argument('-s', '--scale', type= float, help="Image scale", default=1)

    parser.add_argument('-x', '--res-x', type= int, help="Cam Width", default=119)
    parser.add_argument('-y', '--res-y', type= int, help="Cam Height", default=72)

    return parser.parse_args()
        

if __name__ == '__main__':

    args = parse_args()
    
    xy_array = multiprocessing.Array('i', [0,0,0,0,0,0])

   
    # # Create three parallel processes
    p_visualization = multiprocessing.Process(target=visualize_data, args=(args,))

    # Start the processes
    p_visualization.start()

    # Wait for the processes to finish (you may need to manually stop them)
    p_visualization.join()