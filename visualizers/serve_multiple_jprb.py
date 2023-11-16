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
from typing import List
import torchvision
import numpy as np
import time
import torch
import ringbuffer
import learn_shapes2 as learn_shapes

logging.basicConfig(level=logging.DEBUG)

from aestream import UDPInput

# CAMERA_SHAPE = (640, 480)
CAMERA_SHAPE = (1280, 720)

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
            + struct.pack("<f", tensor[4])
            + struct.pack("<f", tensor[5])
            + struct.pack("<f", tensor[6])
        )
        s.sendto(bytes, address)


def sender(host, ports, queue, buffer_size=10):
    sockets = []
    n_cameras = len(ports)
    for port in ports:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
        sockets.append([s, (host, port)])
    print("Sending coordinates to ", ports)
    t = time.time()
    count = 0
    buffer = ringbuffer.RingBuffer((buffer_size, n_cameras, 7))
    try:
        while True:
            tensors = queue.get()
            tensors = buffer.append(tensors).mean(0)
            tensors[:, 3] = tensors[:, 3].astype(np.bool8)
            send_tensors(sockets, tensors)
            count += 1
            if time.time() >= t + 1:
                logging.debug(
                    f"Sending {count} coordinates/s\n"
                    + "\n".join([f"{t[0]},{t[1]},{t[2]},{t[3]}" for t in tensors])
                )
                count = 0
                t = time.time()

    except Exception as e:
        logging.warn("Sender closing", e)
    finally:
        s.close()

def frame_sender(host, ports, queue):

    sockets = []
    for port in ports:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, port))
        sockets.append(s)
    print("Sending frames to ", ports)
    t = time.time()
    count = 0
    try:
        while True:
            tensors = queue.get()
            for i, s in enumerate(sockets):
                s.sendall(tensors[i].tobytes())
            count += 1
            if time.time() >= t + 1:
                logging.debug(f"Sending {count} frames/s\n")
                count = 0
                t = time.time()

    except Exception as e:
        logging.warn("Sender closing", e)
    finally:
        for s in sockets:
            s.close()

def get_prediction(camera, model, state):
    frame = camera.read().to("cuda:0").view(1, 1, 1, CAMERA_SHAPE[0], CAMERA_SHAPE[1])
    out, pose, _, state = model.predict(frame, state)
    return out.cpu(), pose.cpu() * torch.tensor([1, -1]), state

def predictor(
    host,
    checkpoints: List[str],
    queue_out: multiprocessing.Queue,
    queue_frames: multiprocessing.Queue,
    ports_in: List[int],
    interval: float,
):
    assert len(checkpoints) == 3, "Requires three models"
    # sockets = []
    # for port in ports_in:
        # s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # print("Sending stuff to port: [{:.3f}] ".format(port+1700))
        # s.connect((host, int(port+1700)))
        # sockets.append(s)

    try:
        t_0 = time.time()
        t_l = time.time()
        c = 0
        c_total = 0
        state = [None] * len(checkpoints)

        models = [learn_shapes.ShapesModel.load_from_checkpoint(c, map_location="cuda:0").to("cuda:0").eval() for c in checkpoints]
        streams = [
            UDPInput(
                torch.Size((CAMERA_SHAPE[0], CAMERA_SHAPE[1])), device="cpu", port=port#, sum_events=True
            ).start_stream()
            for port in ports_in
        ]

        with torch.inference_mode():
            ports = ",".join([str(p) for p in ports_in])
            logging.info(f"Listening with a {interval * 1000}ms interval on ports {ports}")
            while True:
                t_1 = time.time()
                if t_1 >= t_0 + interval:
                    t_0 = t_1
                    coordinates = []
                    activities = []
                    out = None
                    for i in range(len(models)):
                        out, coo, state[i] = get_prediction(streams[i], models[i], state[i])
                        coordinates.append(coo)
                        activities.append(out.squeeze().numpy())
                    activities = np.array(activities)
                    
                    coordinates = torch.stack(coordinates).squeeze()
                    pose = torch.concat([coordinates, torch.zeros(len(models), 1)], dim=-1) # Add a zero z coordinate
                    pose = pose.numpy() # Cast to numpy
                    if not queue_out.full():
                        alpha = np.array([0,0,0]).reshape(-1, 1)
                        beta = np.array([0,0,0]).reshape(-1, 1)
                        gamma = np.array([0,0,0]).reshape(-1, 1)
                        pres = np.array([1,1,1]).reshape(-1, 1)
                        output = np.concatenate((pose, alpha, beta, gamma, pres), 1)
                        queue_out.put(output, block=False)
                    if not queue_frames.full():
                        queue_frames.put(activities, block=False)

                    c += 1
                    c_total += 1

                if time.time() >= t_l + 1:
                    print(f"Receiving {c}/s")
                    c = 0
                    t_l = time.time()


                # if c_frame >= c_fram_max:
                #     c_frame = 0
                #     # print(ports_in) # [2300, 2301, 2302]
                #     # print(len(tensors)) #3averages
                #         # sockets[k].sendall(struct.pack(">L", size) + data)
                # else:
                #     c_frame +=1

    except Exception as e:
        logging.warn("Predictor closing", e)


def main(args):

    multiprocessing.set_start_method('spawn')

    logging.getLogger().setLevel(args.logging)

    interval = args.interval / 1000  # ms to seconds

    # Sending process
    queue_out = multiprocessing.Queue(100)
    queue_frames = multiprocessing.Queue(100)
    host_visuals = "172.16.222.46"
    host = args.destination
    if args.ports == "default":
        ports_in = [2301, 2302, 2303]
        ports_out = [3001, 3002, 3003]
        ports_frames = [4001, 4002, 4003]
    else:
        ports_in = []
        ports_out = []
        for tup in args.ports:
            port_in, port_out = tup.split(":")
            ports_in.append(int(port_in))
            ports_out.append(int(port_out))

    checkpoints = [''.join(x) for x in args.checkpoints]

    print("Loading models from", ''.join([f"\n- {m}" for m in checkpoints]))

    # Sending
    sending_process = multiprocessing.Process(
        target=sender, args=(host, ports_out, queue_out, args.buffer)
    )
    frame_sending_process = multiprocessing.Process(
        target=frame_sender, args=(host_visuals, ports_frames, queue_frames)
    )

    # Predictor process
    predictor_process = multiprocessing.Process(
        target=predictor, args=(host, checkpoints, queue_out, queue_frames, ports_in, interval)
    )

    sending_process.start()
    frame_sending_process.start()
    predictor_process.start()

    sending_process.join()
    frame_sending_process.join()
    predictor_process.join()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        "Start process for receiving events and emitting poses"
    )
    parser.add_argument(
        "destination", type=str, help="Destination for sending pose over TCP"
    )
    parser.add_argument(
        "--checkpoints", type=list, nargs="+", help="Paths for python model checkpoint file"
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
