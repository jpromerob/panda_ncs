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

IP_SLEIPNER = "172.16.222.28"
IP_NUC = "172.16.222.46"

class cam_info:
    def __init__(self, id, bus,  nbr,  p_sleipner, p_nuc):
        self.id = id
        self.tag = f"inivation {bus} {nbr} dvx"
        self.p_sleipner = p_sleipner
        self.p_nuc = p_nuc

# Function for the processes
def start_streaming(cam_info, mode, undistortion):    


    if mode == "recording":
        cmd_in = f"input file cam{cam_info.id}.aedat4"
    elif mode == "live":
        cmd_in = f"input {cam_info.tag}"
    else:
        print("wrong mode")
        quit()

    cmd_out = f"output udp {IP_SLEIPNER} {cam_info.p_sleipner} {IP_NUC} {cam_info.p_nuc}"

    if undistortion:
        cmd_und = f"resolution 640 480 undistortion cam{cam_info.id}.csv"
        u_message = "WITH undistortion"
    else:
        cmd_und = ""
        u_message = "withOUT undistortion"
    cmd_full = f"/opt/aestream/build/src/aestream {cmd_in} {cmd_out} {cmd_und}"


    time.sleep(2*cam_info.id)
    print(f"Camera #{cam_info.id} ready to stream {u_message}...")
    print(f"stream {cmd_in} {cmd_out} {cmd_und}")
    os.system(cmd_full)

def visualize_data(args):

    window_name = 'Cobotics Workspace'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, (math.ceil(640*2*args.scale), math.ceil(480*2*args.scale)))



    # Stream events from UDP port 3333 (default)
    mgh = 40
    mgi = 20
    mgv = 40
    radius = 10
    fth = 4 # frame thickness
    layout = np.zeros((640*2+mgh*2+mgi,480*2+mgv*2+mgi,3), dtype=np.uint8)

    # Top Margin (for Quadrants)
    layout[mgh-fth:mgh+640+fth,mgv-fth:mgv,:] = 255*np.ones((640+2*fth,fth,3), dtype=np.uint8) # Quadrant Top-Left
    layout[mgh+mgi+640-fth:mgh+mgi+640*2+fth,mgv-fth:mgv,:] = 255*np.ones((640+2*fth,fth,3), dtype=np.uint8) # Quadrant Top-Right
    layout[mgh-fth:mgh+640+fth,mgv+mgi+480-fth:mgv+mgi+480,:] = 255*np.ones((640+2*fth,fth,3), dtype=np.uint8) # Quadrant Bottom-Left
    layout[mgh+mgi+640-fth:mgh+mgi+640*2+fth,mgv+mgi+480-fth:mgv+mgi+480,:] = 255*np.ones((640+2*fth,fth,3), dtype=np.uint8) # Quadrant Bottom-Right

    # Bottom Margin (for Quadrants)
    layout[mgh-fth:mgh+640+fth,mgv+480:mgv+480+fth,:] = 255*np.ones((640+2*fth,fth,3), dtype=np.uint8) # Quadrant Top-Left
    layout[mgh+mgi+640-fth:mgh+mgi+640*2+fth,mgv+480:mgv+480+fth,:] = 255*np.ones((640+2*fth,fth,3), dtype=np.uint8) # Quadrant Top-Right
    layout[mgh-fth:mgh+640+fth,mgv+mgi+480*2:mgv+mgi+480*2+fth,:] = 255*np.ones((640+2*fth,fth,3), dtype=np.uint8) # Quadrant Bottom-Left
    layout[mgh+mgi+640-fth:mgh+mgi+640*2+fth,mgv+mgi+480*2:mgv+mgi+480*2+fth,:] = 255*np.ones((640+2*fth,fth,3), dtype=np.uint8) # Quadrant Bottom-Right

    # Left Margin (for Quadrants)
    layout[mgh-fth:mgh,mgv-fth:mgv+480+fth,:]= 255*np.ones((fth,480+2*fth,3), dtype=np.uint8) # Quadrant Top-Left
    layout[mgh+mgi+640-fth:mgh+mgi+640,mgv-fth:mgv+480+fth,:]= 255*np.ones((fth,480+2*fth,3), dtype=np.uint8) # Quadrant Top-Right
    layout[mgh-fth:mgh,mgv+mgi+480-fth:mgv+mgi+480*2+fth,:]= 255*np.ones((fth,480+2*fth,3), dtype=np.uint8) # Quadrant Bottom-Left
    layout[mgh+mgi+640-fth:mgh+mgi+640,mgv+mgi+480-fth:mgv+mgi+480*2+fth,:]= 255*np.ones((fth,480+2*fth,3), dtype=np.uint8) # Quadrant Bottom-Right

    # Right Margin (for Quadrants)
    layout[mgh+640:mgh+640+fth,mgv-fth:mgv+480+fth,:]= 255*np.ones((fth,480+2*fth,3), dtype=np.uint8) # Quadrant Top-Left
    layout[mgh+mgi+640*2:mgh+mgi+640*2+fth,mgv-fth:mgv+480+fth,:]= 255*np.ones((fth,480+2*fth,3), dtype=np.uint8) # Quadrant Top-Right
    layout[mgh+640:mgh+640+fth,mgv+mgi+480-fth:mgv+mgi+480*2+fth,:]= 255*np.ones((fth,480+2*fth,3), dtype=np.uint8) # Quadrant Bottom-Left
    layout[mgh+mgi+640*2:mgh+mgi+640*2+fth,mgv+mgi+480-fth:mgv+mgi+480*2+fth,:]= 255*np.ones((fth,480+2*fth,3), dtype=np.uint8) # Quadrant Bottom-Right

    frame = layout
    marker = np.zeros((2*radius+1, 2*radius+1, 3), dtype=np.uint8)


    ncs_logo = cv2.imread('ncs_logo_640x480.png') 

    red = (0, 0, 255) 
    ring_th = 5
    with aestream.UDPInput((640, 480), device = 'cpu', port=args.port1) as stream1:
        with aestream.UDPInput((640, 480), device = 'cpu', port=args.port2) as stream2:
            with aestream.UDPInput((640, 480), device = 'cpu', port=args.port3) as stream3:
                
                while True:

                    frame[mgh+0:mgh+640,mgv+mgi+480:mgv+mgi+480*2,0:2] = 255*stream1.read()[:,:,np.newaxis] # cyan
                    frame[mgh+0:mgh+640,mgv+0:mgv+480,1] =  255*stream2.read() # green
                    frame[mgh+mgi+640:mgh+mgi+640*2,mgv+0:mgv+480,1:3] = 255*stream3.read()[:,:,np.newaxis] # yellow

                    frame[mgh+mgi+640:mgh+mgi+640*2,mgv+mgi+480:mgv+mgi+480*2,:] = ncs_logo.transpose(1,0,2)

                    image = cv2.resize(frame.transpose(1,0,2), (math.ceil((640*2+mgh*2+mgi)*args.scale),math.ceil((480*2+mgv*2+mgi)*args.scale)), interpolation = cv2.INTER_AREA)
                    
                    cx = int((mgh+xy_array[0])*args.scale)
                    cy = int((mgv+480+mgi+xy_array[1])*args.scale)
                    cv2.circle(image, (cx, cy), int(radius*args.scale), red, thickness=ring_th)

                    cx = int((mgh+xy_array[2])*args.scale)
                    cy = int((mgv+xy_array[3])*args.scale)
                    cv2.circle(image, (cx, cy), int(radius*args.scale), red, thickness=ring_th)

                    cx = int((mgh+mgi+640+xy_array[4])*args.scale)
                    cy = int((mgv+xy_array[5])*args.scale)
                    cv2.circle(image, (cx, cy), int(radius*args.scale), red, thickness=ring_th)

                    cv2.imshow(window_name, image)
                    cv2.waitKey(1)

def receive_xy(cam_id):
    
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Bind the socket to the receiver's IP and port
    sock.bind((IP_NUC, 4330+cam_id))

    while True:
        data, _ = sock.recvfrom(1024)
        decoded_data = data.decode()
        x, y = map(int, decoded_data.split(','))
        # print(f"receiving {x},{y}")
        xy_array[2*(cam_id-1)] = x
        xy_array[2*(cam_id-1)+1] = 480-y

def print_xy():

    while(True):
        message = "xy_array: "
        for i in range(6):
            message += f" {xy_array[i]} " 
        print(message)
        time.sleep(1)

def parse_args():

    parser = argparse.ArgumentParser(description='Cobotics Workspace Visualizer and Streamer')
    
    parser.add_argument('-c1', '--c1nbr', type=int, help="Camera #1 : lsusb number", default=8)
    parser.add_argument('-c2', '--c2nbr', type=int, help="Camera #2 : lsusb number", default=7)
    parser.add_argument('-c3', '--c3nbr', type=int, help="Camera #3 : lsusb number", default=9)
    
    parser.add_argument('-p1', '--port1', type= int, help="Port for events Cam #1", default=3331)
    parser.add_argument('-p2', '--port2', type= int, help="Port for events Cam #2", default=3332)
    parser.add_argument('-p3', '--port3', type= int, help="Port for events Cam #3", default=3333)

    parser.add_argument('-s', '--scale', type= float, help="Image scale", default=1.8)
    parser.add_argument('-u', '--undistortion', action='store_true', help="Enable undistortion")
    parser.add_argument('-m', '--mode', type=str, help="Mode: live|recording", default="live")

    return parser.parse_args()
        

if __name__ == '__main__':



    args = parse_args()
    xy_array = multiprocessing.Array('i', [0,0,0,0,0,0])
   
    
    # Create an object of the cam_info class
    cam_1 = cam_info(id=1,bus=4, nbr=args.c1nbr, p_sleipner=2301, p_nuc=3331)
    cam_2 = cam_info(id=2,bus=4, nbr=args.c2nbr, p_sleipner=2302, p_nuc=3332)
    cam_3 = cam_info(id=3,bus=4, nbr=args.c3nbr, p_sleipner=2303, p_nuc=3333)

    # Create three parallel processes
    p_streaming_1 = multiprocessing.Process(target=start_streaming, args=(cam_1,args.mode,args.undistortion,))
    p_streaming_2 = multiprocessing.Process(target=start_streaming, args=(cam_2,args.mode,args.undistortion,))
    p_streaming_3 = multiprocessing.Process(target=start_streaming, args=(cam_3,args.mode,args.undistortion,))

    # Create three parallel processes
    p_visualization = multiprocessing.Process(target=visualize_data, args=(args,))
    p_get_xy_cam_1 = multiprocessing.Process(target=receive_xy, args=(1,))
    p_get_xy_cam_2 = multiprocessing.Process(target=receive_xy, args=(2,))
    p_get_xy_cam_3 = multiprocessing.Process(target=receive_xy, args=(3,))
    # p_print_xy = multiprocessing.Process(target=print_xy)

    # Start the processes
    p_visualization.start()
    p_streaming_1.start()
    p_streaming_2.start()
    p_streaming_3.start()
    p_get_xy_cam_1.start()
    p_get_xy_cam_2.start()
    p_get_xy_cam_3.start()
    # p_print_xy.start()

    # Wait for the processes to finish (you may need to manually stop them)
    p_visualization.join()
    p_streaming_1.join()
    p_streaming_2.join()
    p_streaming_3.join()
    p_get_xy_cam_1.join()
    p_get_xy_cam_2.join()
    p_get_xy_cam_3.join()
    # p_print_xy.join()