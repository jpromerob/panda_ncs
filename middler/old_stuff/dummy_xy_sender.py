import socket
import struct  # For packing data into bytes
import math
import time
import multiprocessing

def points_in_circle(radius, center_x, center_y, num_points):
    angle = 0
    angle_increment = 0.4*math.pi/180
    while(True):
        angle += angle_increment
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        yield x, y

def send_data(port):

    xy_generator = points_in_circle(0.8, 0, 0, 360)

    # Define the server address and port (make sure it matches the server configuration)
    server_address = ('172.16.222.48', port)  # Adjust the IP and port as needed

    # Create a socket and connect to the server
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(server_address)

    while(True):

        x, y = next(xy_generator)
        # print(f"({x},{y})")

        # Define data to be sent in the same format as the PayloadSleipner struct
        data = struct.pack('ffff', x, y, 0, 1)
        # Replace the above values with the actual data you want to send

        # Send the data to the server
        client_socket.sendall(data)

        time.sleep(1/120)

    # Close the client socket
    client_socket.close()



if __name__ == "__main__":

    port_1 = 3001
    port_2 = 3002
    port_3 = 3000

    p_streaming_1 = multiprocessing.Process(target=send_data, args=(port_1,))
    p_streaming_2 = multiprocessing.Process(target=send_data, args=(port_2,))
    p_streaming_3 = multiprocessing.Process(target=send_data, args=(port_3,))


    p_streaming_1.start()
    p_streaming_2.start()
    p_streaming_3.start()

    p_streaming_1.join()
    p_streaming_2.join()
    p_streaming_3.join()