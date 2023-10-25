
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
matplotlib.use("TkAgg")
matplotlib.rcParams['toolbar'] = 'None' 
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import socket
import struct
import multiprocessing
import math
import datetime

IP_NUC = '172.16.222.46'

class xyz_estimate:
    def __init__(self, queue):
        self.q = queue
        self.x = []
        self.y = []
        self.z = []
        self.xyz = [-100,-100,-100]

##############################################################################################################################
#                                                          VISUALIZE                                                         #
##############################################################################################################################  

# This function is called periodically from FuncAnimation
def rt_xyz(i, xyz_ncs, xyz_opt, t, axs):


    while not xyz_ncs.q.empty():
        xyz_ncs.xyz = xyz_ncs.q.get(False)

    while not xyz_opt.q.empty():
        xyz_opt.xyz = xyz_opt.q.get(False)

    e_x = round(1000*abs(xyz_ncs.xyz[0]-xyz_opt.xyz[0]),1)
    e_y = round(1000*abs(xyz_ncs.xyz[1]-xyz_opt.xyz[1]),1)
    e_z = round(1000*abs(xyz_ncs.xyz[2]-xyz_opt.xyz[2]),1)

    if xyz_ncs.xyz[0] >= xyz_opt.xyz[0]:
        e_x_sign = "+"
    else:
        e_x_sign = "-"

    if xyz_ncs.xyz[1] >= xyz_opt.xyz[1]:
        e_y_sign = "+"
    else:
        e_y_sign = "-"

    if xyz_ncs.xyz[2] >= xyz_opt.xyz[2]:
        e_z_sign = "+"
    else:
        e_z_sign = "-"

    e_full = round(math.sqrt(e_x**2+e_y**2+e_z**2)/3,1)

    # Add x and y to lists
    t.append(datetime.datetime.now().strftime('%H:%M:%S.%f'))

    xyz_ncs.x.append(xyz_ncs.xyz[0])
    xyz_ncs.y.append(xyz_ncs.xyz[1])
    xyz_ncs.z.append(xyz_ncs.xyz[2])

    xyz_opt.x.append(xyz_opt.xyz[0])
    xyz_opt.y.append(xyz_opt.xyz[1])
    xyz_opt.z.append(xyz_opt.xyz[2])

    # Limit x and y lists to 100 items
    t = t[-100:]

    xyz_ncs.x = xyz_ncs.x[-100:]
    xyz_ncs.y = xyz_ncs.y[-100:]
    xyz_ncs.z = xyz_ncs.z[-100:]

    xyz_opt.x = xyz_opt.x[-100:]
    xyz_opt.y = xyz_opt.y[-100:]
    xyz_opt.z = xyz_opt.z[-100:]

    txt_x = txt_y = txt_z = "No signal"
    if xyz_ncs.x[-1] != -100:
        txt_x = f"x = {round(xyz_ncs.x[-1],3)} [m] ({e_x_sign}{e_x} [mm])"
    if xyz_ncs.y[-1] != -100:
        txt_y = f"y = {round(xyz_ncs.y[-1],3)} [m] ({e_y_sign}{e_y} [mm])"
    if xyz_ncs.z[-1] != -100:
        txt_z = f"z = {round(xyz_ncs.z[-1],3)} [m] ({e_z_sign}{e_z} [mm])"

    # Draw x and y lists
    axs[0].clear()
    axs[0].plot(t, xyz_ncs.x, color='r')
    axs[0].plot(t, xyz_opt.x, color='r', linestyle='--')
    axs[0].text(t[0], 0.1, txt_x, fontsize='xx-large')
    axs[0].xaxis.set_visible(False)
    axs[0].set_ylim([-0.7,0.3])
    axs[0].set_ylabel('x')

    axs[1].clear()
    axs[1].plot(t, xyz_ncs.y, color='g')
    axs[1].plot(t, xyz_opt.y, color='g', linestyle='--')
    axs[1].text(t[0], 0.8, txt_y, fontsize='xx-large')
    axs[1].xaxis.set_visible(False)
    axs[1].set_ylim([0,1])
    axs[1].set_ylabel('y')

    axs[2].clear()
    axs[2].plot(t, xyz_ncs.z, color='b')
    axs[2].plot(t, xyz_opt.z, color='b', linestyle='--')
    axs[2].text(t[0], 1.3, txt_z, fontsize='xx-large')
    axs[2].xaxis.set_visible(False)
    axs[2].set_ylim([0.5,1.5])
    axs[2].set_ylabel('z')

    axs[0].set_title(f"Object Position in Workspace (e={e_full}[mm])", fontsize='xx-large')



def oscilloscope(xyz_ncs_queue, xyz_opt_queue):

    # Create figure for plotting
    fig, axs = plt.subplots(3, figsize=(8.72, 6.18))
    fig.canvas.manager.set_window_title('World Space')


    i = 0

    t = []
    xyz_ncs = xyz_estimate(xyz_ncs_queue)
    xyz_opt = xyz_estimate(xyz_opt_queue)

    # Set up plot to call rt_xyz() function periodically
    ani = animation.FuncAnimation(fig, rt_xyz, fargs=(xyz_ncs, xyz_opt, t, axs), interval=1)
    plt.show()



def get_xyz(xyz_ncs_queue, port_xyz_ncs):

    # Receiver address and port
    receiver_address = (IP_NUC, port_xyz_ncs)

    # Create a UDP socket
    receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver_socket.bind(receiver_address)

    while True:
        # Receive data from the sender
        data, addr = receiver_socket.recvfrom(12)  # Assuming three floats (3 * 4 bytes each)

        # Unpack the received data as three float values
        values = struct.unpack('fff', data)
        xyz_ncs_queue.put([values[0], values[1], values[2]])


    # Close the socket
    receiver_socket.close()


if __name__ == "__main__":
    

    port_xyz_ncs = 6000
    port_xyz_opt = 5999

    xyz_ncs_queue = multiprocessing.Queue()
    xyz_opt_queue = multiprocessing.Queue()

    rt_plot = multiprocessing.Process(target=oscilloscope, args=(xyz_ncs_queue, xyz_opt_queue,))
    xyz_ncs_receiver = multiprocessing.Process(target=get_xyz, args=(xyz_ncs_queue, port_xyz_ncs,))
    xyz_opt_receiver = multiprocessing.Process(target=get_xyz, args=(xyz_opt_queue, port_xyz_opt,))

    rt_plot.start()
    xyz_ncs_receiver.start()
    xyz_opt_receiver.start()

    rt_plot.join()
    xyz_ncs_receiver.join()
    xyz_opt_receiver.join()
