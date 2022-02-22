import socket
import sys
import cv2
import pickle
import numpy as np
import struct ## new
import zlib
import multiprocessing 


''' 
Server that receives 'frames' of events coming from sleipner
'''
def get_image(ip_address, cam_id):

    port_nb = 4000 + cam_id%3 # cam #1 --> 4001 | cam #2 --> 4002 | cam #3 --> 4000

    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    print('Socket created')

    s.bind((ip_address, port_nb))
    print('Socket bind complete')
    s.listen(10)
    print('Socket now listening')

    conn,addr=s.accept()

    data = b""
    payload_size = struct.calcsize(">L")
    # print("payload_size: {}".format(payload_size))
    while True:
        while len(data) < payload_size:
            # print("Recv: {}".format(len(data)))
            data += conn.recv(4096*4)

        # print("Done Recv: {}".format(len(data)))
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]
        # print("msg_size: {}".format(msg_size))
        while len(data) < msg_size:
            data += conn.recv(4096*4)
        frame_data = data[:msg_size]
        data = data[msg_size:]

        frame=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        cv2.imshow('ImageWindow',frame)
        cv2.waitKey(1)


if __name__ == '__main__':

    ip_address = "172.16.222.31"

    bgi_cam_1 = multiprocessing.Process(target=get_image, args=(ip_address, 1,))
    bgi_cam_2 = multiprocessing.Process(target=get_image, args=(ip_address, 2,))
    bgi_cam_3 = multiprocessing.Process(target=get_image, args=(ip_address, 3,))

    bgi_cam_1.start()
    bgi_cam_2.start()
    bgi_cam_3.start()

    bgi_cam_1.join()
    bgi_cam_2.join()
    bgi_cam_3.join()