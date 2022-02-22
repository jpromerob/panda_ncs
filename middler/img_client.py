import cv2
import io
import socket
import struct
import time
import pickle
import zlib
import multiprocessing 
from PIL import Image as im
import numpy as np
import datetime




def send_image(ip_address, cam_id):

    freq = 25
    port_nb = 4000 + cam_id%3 # cam #1 --> 4001 | cam #2 --> 4002 | cam #3 --> 4000

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ip_address, port_nb))
    connection = client_socket.makefile('wb')


    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

    while True:

        start = datetime.datetime.now()
            
        frame = np.random.randint(0,255, (480, 640, 3))
        frame[:,:,cam_id-1] = np.zeros((480,640))
        result, frame = cv2.imencode('.jpg', frame, encode_param)
        data = pickle.dumps(frame, 0)
        size = len(data)

        client_socket.sendall(struct.pack(">L", size) + data)

        stop = datetime.datetime.now()
        diff = stop - start
        t_sleep = (1000000/freq - int(diff.microseconds))/1000000
        try:
            time.sleep(t_sleep)
        except:
            print('Too slow ... ')



if __name__ == '__main__':

    ip_address = "172.16.222.31"

    bgi_cam_1 = multiprocessing.Process(target=send_image, args=(ip_address, 1,))
    bgi_cam_2 = multiprocessing.Process(target=send_image, args=(ip_address, 2,))
    bgi_cam_3 = multiprocessing.Process(target=send_image, args=(ip_address, 3,))

    bgi_cam_1.start()
    bgi_cam_2.start()
    bgi_cam_3.start()

    bgi_cam_1.join()
    bgi_cam_2.join()
    bgi_cam_3.join()