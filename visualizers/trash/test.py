
import aestream
import time
import cv2
import pdb
import numpy as np
import math
import argparse
import csv
import os


def parse_args():

    parser = argparse.ArgumentParser(description='Cobotics Workspace Visualizer')
    parser.add_argument('-s', '--scale', type= float, help="Image scale", default=1)

    return parser.parse_args()

if __name__ == '__main__':


    args = parse_args()


    window_name = 'Cobotics Workspace'

    ncs_logo = cv2.resize(cv2.imread('ncs_logo_640_480.jpg', cv2.IMREAD_UNCHANGED), (math.ceil(640*args.scale), math.ceil(480*args.scale))).transpose(1,0,2)
    cv2.namedWindow(window_name)
    cv2.imshow(window_name, ncs_logo)

    # Wait for a key press indefinitely or for a specified time (in milliseconds)
    cv2.waitKey(0)

    # Close the image window
    cv2.destroyAllWindows()