import multiprocessing
import time
import os
import argparse

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

def parse_args():

    parser = argparse.ArgumentParser(description='Cobotics Camera Streamer')
    parser.add_argument('-m', '--mode', type=str, help="Mode: live|recording", default="live")
    parser.add_argument('-c1', '--c1nbr', type=int, help="Camera #1 : lsusb number", default=8)
    parser.add_argument('-c2', '--c2nbr', type=int, help="Camera #2 : lsusb number", default=7)
    parser.add_argument('-c3', '--c3nbr', type=int, help="Camera #3 : lsusb number", default=9)
    parser.add_argument('-u', '--undistortion', action='store_true', help="Enable undistortion")


    return parser.parse_args()

if __name__ == '__main__':


    args = parse_args()
    
    # Create an object of the cam_info class
    cam_1 = cam_info(id=1,bus=4, nbr=args.c1nbr, p_sleipner=2301, p_nuc=3331)
    cam_2 = cam_info(id=2,bus=4, nbr=args.c2nbr, p_sleipner=2302, p_nuc=3332)
    cam_3 = cam_info(id=3,bus=4, nbr=args.c3nbr, p_sleipner=2300, p_nuc=3333)

    # Create three parallel processes
    p1 = multiprocessing.Process(target=start_streaming, args=(cam_1,args.mode,args.undistortion,))
    p2 = multiprocessing.Process(target=start_streaming, args=(cam_2,args.mode,args.undistortion,))
    p3 = multiprocessing.Process(target=start_streaming, args=(cam_3,args.mode,args.undistortion,))

    # Start the processes
    p1.start()
    p2.start()
    p3.start()

    # Wait for the processes to finish (you may need to manually stop them)
    p1.join()
    p2.join()
    p3.join()