import multiprocessing
import socket
import time
import random

def sender_process(ip, port):
    out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        x = random.randint(0, 640)
        y = random.randint(0, 480)       
        message = f"{x},{y}"
        print(f"sending {message}")
        out_sock.sendto(message.encode(), (ip, port))
        time.sleep(0.2)

DST_IP = '172.16.222.46'

if __name__ == "__main__":

    p1 = 4331  # Change to the port number you want to use
    p2 = 4332  # Change to the port number you want to use
    p3 = 4333  # Change to the port number you want to use

    sender1 = multiprocessing.Process(target=sender_process, args=(DST_IP, p1))
    sender2 = multiprocessing.Process(target=sender_process, args=(DST_IP, p2))
    sender3 = multiprocessing.Process(target=sender_process, args=(DST_IP, p3))
    sender1.start()
    sender2.start()
    sender3.start()
    sender1.join() 
    sender2.join() 
    sender3.join() 
