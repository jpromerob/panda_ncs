import socket
import numpy as np
import time
import argparse

def send_array(tcp_ip, tcp_port, res_x, res_y):
    # Create a TCP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connect to the server
    sock.connect((tcp_ip, tcp_port))
    print(f"Connected to the server on port {tcp_port}.")

    try:
        while True:
            # Generate a NumPy array (replace this line with your array creation logic)
            # my_array = (np.random.rand(res_x, res_y) * 1e-20/255).astype(np.float64)
            # my_array = (np.ones((res_x, res_y))*1e-20/255).astype(np.float64)
            my_array = (np.ones((res_x, res_y))*255).astype(np.float64)

            # Flatten the array data
            array_data = my_array.flatten()

            # Send the array data
            sock.sendall(array_data.tobytes())

            time.sleep(1 / 60)

    except KeyboardInterrupt:
        print("KeyboardInterrupt: Closing the connection.")
    finally:
        # Close the socket
        sock.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="TCP Client for sending NumPy arrays")
    parser.add_argument("--tcp_ip", type=str, default="172.16.222.46", help="Server's IP address")
    parser.add_argument("--tcp_port", type=int, default=4001, help="Server's port number")
    parser.add_argument("--res_x", type=int, default=119, help="Array width")
    parser.add_argument("--res_y", type=int, default=72, help="Array height")

    args = parser.parse_args()

    print(f"TCP PORT: {args.tcp_port}")

    send_array(args.tcp_ip, args.tcp_port, args.res_x, args.res_y)
