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

    while(True):
        # Reshape the array to its original shape (640x480)

        array_1 = np.frombuffer(cam_array_1.get_obj(), dtype=np.float64).reshape((args.res_x, args.res_y))
        array_2 = np.frombuffer(cam_array_2.get_obj(), dtype=np.float64).reshape((args.res_x, args.res_y))
        array_3 = np.frombuffer(cam_array_3.get_obj(), dtype=np.float64).reshape((args.res_x, args.res_y))


        max1 = np.max(array_1)
        max2 = np.max(array_2)
        max3 = np.max(array_3)


        m1 = params[0] #np.max(array_1)
        m2 = params[1] #np.max(array_2)
        m3 = params[2] #np.max(array_3)

        # max_bar_height = 1000

        # s1 = min(max_bar_height,np.sum(array_1)/1e-11)
        # s2 = min(max_bar_height,np.sum(array_2)/1e-11)
        # s3 = min(max_bar_height,np.sum(array_3)/1e-11)

        # bar_1_height = max(1,int(rxy[1]*s1/max_bar_height))
        # bar_2_height = max(1,int(rxy[1]*s2/max_bar_height))
        # bar_3_height = max(1,int(rxy[1]*s3/max_bar_height))

        if max1>0 and max2>0 and max3>0:
            print(f"max 1 : {max1}")
            print(f"\tmax 2 : {max2}")
            print(f"\t\tmax 3 : {max3}")
        #     print(f"Sum #1: {bar_1_height}")
        #     print(f"\tSum #2: {bar_2_height}")
        #     print(f"\t\tSum #3: {bar_3_height}")
        
        if m1 > 0:
            array_1 = array_1 * (m1)
        if m2 > 0:
            array_2 = array_2 * (m2)
        if m3 > 0:
            array_3 = array_3 * (m3)



                

        frame[mgh+0:mgh+rxy[0],mgv+mgi+rxy[1]:mgv+mgi+rxy[1]*2,0:2] = 255*array_1[:,:,np.newaxis] # cyan
        frame[mgh+0:mgh+rxy[0],mgv+0:mgv+rxy[1],1] =  255*array_2 # green
        frame[mgh+mgi+rxy[0]:mgh+mgi+rxy[0]*2,mgv+0:mgv+rxy[1],1:3] = 255*array_3[:,:,np.newaxis] # yellow

        image = cv2.resize(frame.transpose(1,0,2), (math.ceil((rxy[0]*2+mgh*2+mgi)*args.scale),math.ceil((rxy[1]*2+mgv*2+mgi)*args.scale)), interpolation = cv2.INTER_AREA)
        
        # cv2.rectangle(image, top_r_1, bot_r_1, (255,255,255), thickness=-1)
        # cv2.rectangle(image, top_r_2, bot_r_2, (255,255,255), thickness=-1)
        # cv2.rectangle(image, top_r_3, bot_r_3, (255,255,255), thickness=-1)


        # cv2.rectangle(image, (top_r_1[0], bot_r_1[1]+bar_1_height), bot_r_1, (0,255,0), thickness=-1)
        # cv2.rectangle(image, (top_r_2[0], bot_r_2[1]+bar_2_height), bot_r_2, (255,255,0), thickness=-1)
        # cv2.rectangle(image, (top_r_3[0], bot_r_3[1]+bar_3_height), bot_r_3, (0,255,255), thickness=-1)

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




        if params[0] != data[0]:
            print(f"New scale: {data[0]}")
            params[0] = data[0]
        params[1] = data[1]
        params[2] = data[2]


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

    params = multiprocessing.Array('d', [0.0,0.0,0.0])
    

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