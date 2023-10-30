###
##
## This script reads DVS frames from a number of sockets, sends them to a GPU model and emits a pose estimate to a TCP server
##
###

import argparse
import io
import logging
import multiprocessing
import socket
import struct
from typing import List, Optional
import numpy as np
import time
import torch
import coordinates
from model_inference import ModelInference
from dsnt import SpotModel
import ringbuffer

from aestream import UDPInput
from presence import PresenceModel

import cv2
import pickle
import zlib
from PIL import Image as im

CAMERA_SHAPE = (640, 480)

def send_tensors(sockets, tensors):

    for i in range(len(sockets)):
        tensor = tensors[i]
        s = sockets[i][0]
        address = sockets[i][1]
        bytes = (
            struct.pack("<f", tensor[0])
            + struct.pack("<f", tensor[1])
            + struct.pack("<f", tensor[2])
            + struct.pack("<f", tensor[3])
        )

        s.sendto(bytes, address)


def sender(host, ports, queue, buffer=10):
    sockets = []
    n_cameras = len(ports)
    for port in ports:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
        sockets.append([s, (host, port)])
    print("Sending averages to ", ports)
    t = time.time()
    count = 0
    buffer = ringbuffer.RingBuffer((buffer, n_cameras, 4))
    try:
        while True:
            tensors = queue.get()
            tensors = buffer.append(tensors).mean(0)
            tensors[:, 3] = tensors[:, 3].astype(np.bool8)
            send_tensors(sockets, tensors)
            count += 1
            if time.time() >= t + 1:
                logging.debug(
                    f"Sending {count}/s\n"
                    + "\n".join([f"{t[0]},{t[1]},{t[2]},{t[3]}" for t in tensors])
                )
                count = 0
                t = time.time()

    except Exception as e:
        logging.warn("Sender closing", e)
    finally:
        s.close()


def predictor(
    host,
    checkpoint: str,
    queue_out: multiprocessing.Queue,
    ports_in: List[int],
    interval: float,
):

    # sockets = []
    # for port in ports_in:
        # s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # print("Sending stuff to port: [{:.3f}] ".format(port+1700))
        # s.connect((host, int(port+1700)))
        # sockets.append(s)

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

    try:
        t_0 = time.time()
        t_l = time.time()
        c = 0
        c_total = 0
        c_frame = 0
        freq_frame = 25
        c_fram_max = int((1/interval)/freq_frame)

        # model = ModelInference(checkpoint, "cuda")
        model = SpotModel.load_from_checkpoint(checkpoint, batch_size=len(ports_in))
        presence = PresenceModel(threshold=200).to("cuda:0")
        streams = [
            UDPInput(
                torch.Size((640, 480)), device="cpu", port=port#, sum_events=True
            ).start_stream()
            for port in ports_in
        ]

        ports = ",".join([str(p) for p in ports_in])
        logging.info(f"Listening with a {interval * 1000}ms interval on ports {ports}")
        while True:
            t_1 = time.time()
            if t_1 >= t_0 + interval:
                t_0 = t_1
                tensors = [
                        torch.tensor(s.read(), device="cuda:0").view(CAMERA_SHAPE[0], CAMERA_SHAPE[1]) for s in streams
                ]


                tensor = torch.stack(tensors).view(
                    len(ports_in), 1, CAMERA_SHAPE[0], CAMERA_SHAPE[1]
                )
                pose = model(tensor)
                pres = presence(tensor).reshape(-1, 1)
                if not queue_out.full():
                    output = np.concatenate([pose, pres], 1)
                    queue_out.put(output, block=False)

                c += 1
                c_total += 1

            if time.time() >= t_l + 1:
                print(f"Receiving {c}/s")
                c = 0
                t_l = time.time()


            if c_frame >= c_fram_max:
                c_frame = 0
                # print(ports_in) # [2300, 2301, 2302]
                # print(len(tensors)) #3
                # print(tensors[1].size()) # torch.Size([640, 480])

                for k in range(len(ports_in)):
                    result, frame = cv2.imencode('.jpg', tensors[k].cpu().numpy() , encode_param)
                    data = pickle.dumps(frame, 0)
                    size = len(data)
                    # sockets[k].sendall(struct.pack(">L", size) + data)
            else:
                c_frame +=1

    except Exception as e:
        logging.warn("Predictor closing", e)


def main(args):

    multiprocessing.set_start_method('spawn')

    logging.getLogger().setLevel(args.logging)

    interval = args.interval / 1000  # ms to seconds

    # Sending process
    queue_out = multiprocessing.Queue(100)
    host = args.destination
    if args.ports == "default":
        ports_in = [2301, 2302, 2303]
        ports_out = [3001, 3002, 3003]
    else:
        ports_in = []
        ports_out = []
        for tup in args.ports:
            port_in, port_out = tup.split(":")
            ports_in.append(int(port_in))
            ports_out.append(int(port_out))

    sending_process = multiprocessing.Process(
        target=sender, args=(host, ports_out, queue_out, args.buffer)
    )

    # Predictor process
    predictor_process = multiprocessing.Process(
        target=predictor, args=(host, args.checkpoint, queue_out, ports_in, interval)
    )

    sending_process.start()
    predictor_process.start()

    sending_process.join()
    predictor_process.join()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        "Start process for receiving events and emitting poses"
    )
    parser.add_argument(
        "destination", type=str, help="Destination for sending pose over TCP"
    )
    parser.add_argument(
        "checkpoint", type=str, help="Path for python model checkpoint file"
    )
    parser.add_argument("--device", type=str, default="cuda", help="PyTorch device")
    parser.add_argument("--ports", nargs="+", default="default")
    parser.add_argument(
        "--interval",
        type=int,
        default=2,
        help="Time interval between predictions in ms",
    )
    parser.add_argument(
        "--visualize", action="store_true", default=False, help="Host sending events"
    )
    parser.add_argument("--logging", type=str, default="DEBUG", help="Log level")
    parser.add_argument(
        "--buffer", type=int, default=1, help="Buffer size, 1 == no buffer"
    )
    args = parser.parse_args()
    main(args)
