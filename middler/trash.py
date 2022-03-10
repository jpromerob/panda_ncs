

import argparse


import xml.etree.ElementTree as ET
import re
# import os
import numpy as np
# import time
# import cv2 as cv
# import glob, os
# import json
# import matplotlib.image as mpimg
# from numpy import genfromtxt
# import matplotlib.pyplot as plt
# import matplotlib.mlab as mlab
# from matplotlib.colors import LinearSegmentedColormap

def load_params(filename, source):

    tree = ET.parse(filename)
    root = tree.getroot()

    nb_cams = 3
    param_cat = ["pose", "focl"]
    nb_params = [6,2]

    cam_poses = np.zeros((3,nb_params[0]))
    focl = np.zeros((3,nb_params[1]))
    μ = np.zeros(3)
    Σ = np.zeros((3,3))

    # Regular expression to get values out of *.xml
    rescinot = re.compile('[-+]?[\d]+\.?[\d]*')

    # Getting camera parameters (x3)
    for j in range(nb_cams):

        i = 0
        for k in nb_params:


            # Get Camera Matrix 
            data = np.array(re.findall(rescinot, root[0][j][i].text))

            # Refactoring matrices and coefficients

            if param_cat[i] == "pose":
                cam_poses[j,:] = data.astype(np.float64).reshape(k)

            if param_cat[i] == "focl":
                focl[j,:] = data.astype(np.float64).reshape(2)

            i += 1

    focl = np.transpose(focl)

    # Getting Tuning
    if source == "dvs":
        j = 0
    else:
        j = 1
    data = np.array(re.findall(rescinot, root[1][j].text))
    tuning = data.astype(np.float64).reshape(7)

    μ[2] = tuning[0]
    Σ[0,0] = tuning[1]
    Σ[1,1] = tuning[2]
    Σ[2,2] = tuning[3]
    offset = tuning[4:7]


    print(cam_poses)
    print(focl)
    print(μ)
    print(Σ)
    print(offset)
    
    return cam_poses, focl

if __name__ == "__main__":

    parser = argparse.ArgumentParser("Start process for receiving events and emitting poses")

    parser.add_argument("destination", type=str, help="Destination for sending pose over TCP")
    parser.add_argument("checkpoint", type=str, help="Path for python model checkpoint file")

    parser.add_argument("--device", type=str, default="cuda", help="PyTorch device")
    parser.add_argument("--ports", nargs="+", default="default")
    parser.add_argument("--interval", type=int, default=2, help="Time interval between predictions in ms")
    parser.add_argument("--visualize", action="store_true", default=False, help="Host sending events")
    parser.add_argument("--logging", type=str, default="DEBUG", help="Log level")
    parser.add_argument("--buffer", type=int, default=1, help="Buffer size, 1 == no buffer")

    args = parser.parse_args()

    print(args)

    cam_poses, focl = load_params('params.xml', 'dvs')