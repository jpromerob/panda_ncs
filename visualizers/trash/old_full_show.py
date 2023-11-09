
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

    window_name = 'Cobotics Workspace'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, (math.ceil(640*2*args.scale), math.ceil(480*2*args.scale)))



    # Stream events from UDP port 3333 (default)
    mgh = 40
    mgi = 20
    mgv = 40
    radius = 10
    frame = np.zeros((640*2+mgh*2+mgi,480*2+mgv*2+mgi,3))
    marker = np.zeros((2*radius+1, 2*radius+1, 3))


    red = (0, 0, 255) 
    with aestream.UDPInput((640, 480), device = 'cpu', port=args.port1) as stream1:
        with aestream.UDPInput((640, 480), device = 'cpu', port=args.port2) as stream2:
            with aestream.UDPInput((640, 480), device = 'cpu', port=args.port3) as stream3:
                
                while True:

                    frame[mgh+0:mgh+640,mgv+mgi+480:mgv+mgi+480*2,0:2] = stream1.read()[:,:,np.newaxis] # cyan
                    frame[mgh+0:mgh+640,mgv+0:mgv+480,1] =  stream2.read() # green
                    frame[mgh+mgi+640:mgh+mgi+640*2,mgv+0:mgv+480,1:3] = stream3.read()[:,:,np.newaxis] # yellow

                    

                    image = cv2.resize(frame.transpose(1,0,2), (math.ceil((640*2+mgh*2+mgi)*args.scale),math.ceil((480*2+mgv*2+mgi)*args.scale)), interpolation = cv2.INTER_AREA)
                    
                    cx = int((mgh+xy_array[0])*args.scale)
                    cy = int((mgv+480+mgi+xy_array[1])*args.scale)
                    cv2.circle(image, (cx, cy), int(radius*args.scale), red, thickness=2)

                    cx = int((mgh+xy_array[2])*args.scale)
                    cy = int((mgv+xy_array[3])*args.scale)
                    cv2.circle(image, (cx, cy), int(radius*args.scale), red, thickness=2)

                    cx = int((mgh+mgi+640+xy_array[4])*args.scale)
                    cy = int((mgv+xy_array[5])*args.scale)
                    cv2.circle(image, (cx, cy), int(radius*args.scale), red, thickness=2)

                    cv2.imshow(window_name, image)
                    cv2.waitKey(1)

def receive_xy(cam_id):
    
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Bind the socket to the receiver's IP and port
    sock.bind(("172.16.222.46", 4330+cam_id))

    while True:
        data, _ = sock.recvfrom(1024)
        decoded_data = data.decode()
        x, y = map(int, decoded_data.split(','))
        # print(f"receiving {x},{y}")
        xy_array[2*(cam_id-1)] = x
        xy_array[2*(cam_id-1)+1] = y

def print_xy():

    while(True):
        message = "xy_array: "
        for i in range(6):
            message += f" {xy_array[i]} " 
        print(message)
        time.sleep(1)

def parse_args():

    parser = argparse.ArgumentParser(description='Cobotics Workspace Visualizer')
    parser.add_argument('-p1', '--port1', type= int, help="Port for events", default=3331)
    parser.add_argument('-p2', '--port2', type= int, help="Port for events", default=3332)
    parser.add_argument('-p3', '--port3', type= int, help="Port for events", default=3333)
    parser.add_argument('-s', '--scale', type= float, help="Image scale", default=1.8)

    return parser.parse_args()
        

if __name__ == '__main__':


    args = parse_args()
    xy_array = multiprocessing.Array('i', [0,0,0,0,0,0])

    

    # Create three parallel processes
    p_visualization = multiprocessing.Process(target=visualize_data, args=(args,))
    p_get_xy_cam_1 = multiprocessing.Process(target=receive_xy, args=(1,))
    p_get_xy_cam_2 = multiprocessing.Process(target=receive_xy, args=(2,))
    p_get_xy_cam_3 = multiprocessing.Process(target=receive_xy, args=(3,))
    p_print_xy = multiprocessing.Process(target=print_xy)

    # Start the processes
    p_visualization.start()
    p_get_xy_cam_1.start()
    p_get_xy_cam_2.start()
    p_get_xy_cam_3.start()
    p_print_xy.start()

    # Wait for the processes to finish (you may need to manually stop them)
    p_visualization.join()
    p_get_xy_cam_1.join()
    p_get_xy_cam_2.join()
    p_get_xy_cam_3.join()
    p_print_xy.join()