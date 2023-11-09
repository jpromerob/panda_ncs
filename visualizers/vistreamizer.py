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

IP_NUC = "172.16.222.46"


class cam_info:
    def __init__(self, type, x, y, id, bus,  nbr,  p_sleipner, p_nuc):
        self.type = type
        self.res_x = x
        self.res_y = y
        self.id = id
        if self.type == "prophesee":
            if id == 1:
                self.serial = "Prophesee:hal_plugin_prophesee:00050948"
            if id == 2:
                self.serial = "Prophesee:hal_plugin_prophesee:00050945"
            if id == 3:
                self.serial = "Prophesee:hal_plugin_prophesee:00050947"
            self.tag = f"prophesee {self.serial}"
        if self.type == "inivation":
            self.serial = ""
            self.tag = f"inivation {bus} {nbr} dvx"
        self.p_sleipner = p_sleipner
        self.p_nuc = p_nuc

# Function for the processes
def start_streaming(cam_info, mode, undistortion, ip_processor):

    if mode == "recording":
        cmd_in = f"input file cam{cam_info.id}.aedat4"
    elif mode == "live":
        cmd_in = f"input {cam_info.tag}"
    else:
        print("wrong mode")
        quit()

    cmd_out = f"output udp {ip_processor} {cam_info.p_sleipner} {IP_NUC} {cam_info.p_nuc}"

    if undistortion:
        if cam_info.type == "Inivation":
            cmd_und = f"resolution {cam_info.res_x} {cam_info.res_y} undistortion {cam_info.type}/cam{cam_info.id}.csv"
        else:
            cmd_und = f""
        u_message = "WITH undistortion"
    else:
        cmd_und = ""
        u_message = "withOUT undistortion"
    cmd_full = f"/opt/aestream/build/src/aestream {cmd_in} {cmd_out} {cmd_und}"


    time.sleep(5*cam_info.id)
    print(f"Camera #{cam_info.id} ready to stream {u_message}...")
    print(f"stream {cmd_in} {cmd_out} {cmd_und}")
    os.system(cmd_full)

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

    red = (0, 0, 255) 
    orange = (0, 128, 255) 
    ring_th = 5
    with aestream.UDPInput((rxy[0], rxy[1]), device = 'cpu', port=args.port1) as stream1:
        with aestream.UDPInput((rxy[0], rxy[1]), device = 'cpu', port=args.port2) as stream2:
            with aestream.UDPInput((rxy[0], rxy[1]), device = 'cpu', port=args.port3) as stream3:
                
                while True:

                    frame[mgh+0:mgh+rxy[0],mgv+mgi+rxy[1]:mgv+mgi+rxy[1]*2,0:2] = 255*stream1.read()[:,:,np.newaxis] # cyan
                    frame[mgh+0:mgh+rxy[0],mgv+0:mgv+rxy[1],1] =  255*stream2.read() # green
                    frame[mgh+mgi+rxy[0]:mgh+mgi+rxy[0]*2,mgv+0:mgv+rxy[1],1:3] = 255*stream3.read()[:,:,np.newaxis] # yellow

                    # frame[mgh+mgi+rxy[0]:mgh+mgi+rxy[0]*2,mgv+mgi+rxy[1]:mgv+mgi+rxy[1]*2,:] = ncs_logo.transpose(1,0,2)

                    image = cv2.resize(frame.transpose(1,0,2), (math.ceil((rxy[0]*2+mgh*2+mgi)*args.scale),math.ceil((rxy[1]*2+mgv*2+mgi)*args.scale)), interpolation = cv2.INTER_AREA)
                    
                    cx = int((mgh+xy_array[0])*args.scale)
                    cy = int((mgv+rxy[1]+mgi+xy_array[1])*args.scale)
                    cv2.circle(image, (cx, cy), int(radius*args.scale), red, thickness=ring_th)

                    cx = int((mgh+xy_array[2])*args.scale)
                    cy = int((mgv+xy_array[3])*args.scale)
                    cv2.circle(image, (cx, cy), int(radius*args.scale), red, thickness=ring_th)

                    cx = int((mgh+mgi+rxy[0]+xy_array[4])*args.scale)
                    cy = int((mgv+xy_array[5])*args.scale)
                    cv2.circle(image, (cx, cy), int(radius*args.scale), red, thickness=ring_th)


                    # center of top left
                    cv2.circle(image, (int(mgh+rxy[0]/2), int(mgv+rxy[1]/2)), int(radius*args.scale), orange, thickness=ring_th)
                    # center of top right
                    cv2.circle(image, (int(mgh+mgi+rxy[0]*3/2), int(mgv+rxy[1]/2)), int(radius*args.scale), orange, thickness=ring_th)
                    # center of bottom left
                    cv2.circle(image, (int(mgh+rxy[0]/2), int(mgv+mgi+rxy[1]*3/2)), int(radius*args.scale), orange, thickness=ring_th)

                    cv2.imshow(window_name, image)
                    cv2.waitKey(1)

def receive_xy(cam_id, res_y):
    
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
        xy_array[2*(cam_id-1)+1] = res_y-y

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

    parser.add_argument('-s', '--scale', type= float, help="Image scale", default=1)


    parser.add_argument('-ip', '--ip-processor', type= str, help="IP of SNN processor", default="172.16.222.27")
    parser.add_argument('-t', '--cam-type', type= str, help="Type of Camera", default="prophesee")
    
    parser.add_argument('-u', '--undistortion', action='store_true', help="Enable undistortion")
    parser.add_argument('-m', '--mode', type=str, help="Mode: live|recording", default="live")

    return parser.parse_args()
        

if __name__ == '__main__':

    # export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/openeb/build/lib
    print("Don't forget to do: \nexport LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/openeb/build/lib")
    args = parse_args()
    if args.cam_type == "inivation":
        args.res_x = 640
        args.res_y = 480
    elif args.cam_type == "prophesee":
        args.res_x = 1280
        args.res_y = 720
    else:
        print("Build it yourself!")
        quit()
    
    xy_array = multiprocessing.Array('i', [0,0,0,0,0,0])
   
    # Create an object of the cam_info class
    cam_1 = cam_info(type=args.cam_type, x=args.res_x, y=args.res_y, id=1, bus=4, nbr=args.c1nbr, p_sleipner=2301, p_nuc=3331)
    cam_2 = cam_info(type=args.cam_type, x=args.res_x, y=args.res_y, id=2, bus=4, nbr=args.c2nbr, p_sleipner=2302, p_nuc=3332)
    cam_3 = cam_info(type=args.cam_type, x=args.res_x, y=args.res_y, id=3, bus=4, nbr=args.c3nbr, p_sleipner=2303, p_nuc=3333)

    # Create three parallel processes
    p_streaming_1 = multiprocessing.Process(target=start_streaming, args=(cam_1,args.mode,args.undistortion,args.ip_processor))
    p_streaming_2 = multiprocessing.Process(target=start_streaming, args=(cam_2,args.mode,args.undistortion,args.ip_processor))
    p_streaming_3 = multiprocessing.Process(target=start_streaming, args=(cam_3,args.mode,args.undistortion,args.ip_processor))

    # Create three parallel processes
    p_visualization = multiprocessing.Process(target=visualize_data, args=(args,))
    p_get_xy_cam_1 = multiprocessing.Process(target=receive_xy, args=(1,args.res_y,))
    p_get_xy_cam_2 = multiprocessing.Process(target=receive_xy, args=(2,args.res_y,))
    p_get_xy_cam_3 = multiprocessing.Process(target=receive_xy, args=(3,args.res_y,))

    # Start the processes
    p_visualization.start()
    p_streaming_1.start()
    p_streaming_2.start()
    p_streaming_3.start()
    p_get_xy_cam_1.start()
    p_get_xy_cam_2.start()
    p_get_xy_cam_3.start()

    # Wait for the processes to finish (you may need to manually stop them)
    p_visualization.join()
    p_streaming_1.join()
    p_streaming_2.join()
    p_streaming_3.join()
    p_get_xy_cam_1.join()
    p_get_xy_cam_2.join()
    p_get_xy_cam_3.join()