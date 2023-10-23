
import socket
import struct
import multiprocessing
import numpy as np
import math
import pdb


'''
This function sets the camera poses based on manual readings from optitrack (using camera marker 'hat')

# CAM1 -0.136	0.923	1.404	-70.909	11.156	-23.797
# CAM2 -0.633	0.953	1.414	-57.749	-34.985	-11.712
# CAM3 -0.701	0.938	0.554	171.656	-52.186	122.931

'''
def set_cam_poses():

    cam_poses = np.zeros((3,6))

    # Cam 1
    cam_poses[0,0] = -0.136 # cam1:cx
    cam_poses[0,1] = 0.923 # cam1:cy
    cam_poses[0,2] = 1.404 # cam1:cz
    cam_poses[0,3] = (math.pi/180)*(-70.909) # cam1:alpha
    cam_poses[0,4] = (math.pi/180)*(11.156) # cam1:beta
    cam_poses[0,5] = (math.pi/180)*(-23.797) # cam1:gamma

    # Cam 2
    cam_poses[1,0] = -0.633 # cam2:cx
    cam_poses[1,1] = 0.953 # cam2:cy
    cam_poses[1,2] = 1.414 # cam2:cz
    cam_poses[1,3] = (math.pi/180)*(-57.749) # cam2:alpha
    cam_poses[1,4] = (math.pi/180)*(-34.985) # cam2:beta
    cam_poses[1,5] = (math.pi/180)*(-11.712) # cam2:gamma

    # Cam 3
    cam_poses[2,0] = -0.701 # cam3:cx
    cam_poses[2,1] = 0.938 # cam3:cy
    cam_poses[2,2] = 0.554 # cam3:cz
    cam_poses[2,3] = (math.pi/180)*(171.656) # cam3:alpha
    cam_poses[2,4] = (math.pi/180)*(-52.186) # cam3:beta
    cam_poses[2,5] = (math.pi/180)*(122.931) # cam3:gamma

    return cam_poses

def set_focal_lengths():

    focl = np.zeros((2,3))

    focl[0,0] = 657.695274 
    focl[0,1] = 689.301596 
    focl[0,2] = 775.474293 
    focl[1,0] = 656.424014 
    focl[1,1] = 686.973465 
    focl[1,2] = 774.587397 

    return focl


''' Translation Matrices'''
def get_transmats(cam_poses):
    
    mat_tran = np.zeros((4,4,3))
    for i in range(3): # Cam 1, 2, 3
        
        cx = cam_poses[i,0]
        cy = cam_poses[i,1]
        cz = cam_poses[i,2]

        # Transformation matrices (translation + rotations around x, y, z)
        mat_tran[:,:,i] = np.array([[1,0,0,cx],
                                    [0,1,0,cy],
                                    [0,0,1,cz],
                                    [0,0,0,1]])
        
    return mat_tran
    
    
    
'''Rotation Matrices'''
def get_rotmats(cam_poses):
    
    mat_rota = np.zeros((4,4,3))
    for i in range(3): # Cam 1, 2, 3
        
        alpha = cam_poses[i,3]
        beta = cam_poses[i,4] 
        gamma = cam_poses[i,5]


        mat_rotx = np.array([[1,0,0,0],
                             [0,math.cos(alpha), -math.sin(alpha),0],
                             [0, math.sin(alpha), math.cos(alpha),0],
                             [0,0,0,1]])

        mat_roty = np.array([[math.cos(beta), 0, math.sin(beta),0],
                             [0,1,0,0],
                             [-math.sin(beta), 0, math.cos(beta),0],
                             [0, 0, 0, 1]])


        mat_rotz = np.array([[math.cos(gamma), -math.sin(gamma), 0, 0],
                             [math.sin(gamma), math.cos(gamma),0, 0],
                             [0,0,1,0],
                             [0,0,0,1]])

        # General rotation matrix
        mat_rota[:,:,i] = mat_rotz.dot(mat_roty).dot(mat_rotx)    
    
    return mat_rota

'''
This function defines ground truth (for a point in space)
'''
def define_object_pose(c2w, ground_truth):
    
    perspective = np.zeros((4,3)) # coordinates|cameras
    # Checking output of each camera
    for i in range(3): # Cam 1, 2, 3
        w2c = np.linalg.inv(c2w[:,:,i])
        perspective[:, i] = w2c.dot(ground_truth)

    return perspective[0:3,:]


'''
This functions determines the angular 'distance' between camera and object in planez XZ and YZ
'''
def get_angles_from_pos(obj_pose):
    
    angles = np.zeros(2)
    
    angles[0] = (180/math.pi)*math.atan2((obj_pose[0]),(obj_pose[2])) + 180 # delta_x/delta_z
    angles[1] = (180/math.pi)*math.atan2((obj_pose[1]),(obj_pose[2])) + 180 # delta_y/delta_z

    if(angles[0]>180):
        angles[0] = 360-angles[0]
    if(angles[1]>180):
        angles[1] = 360-angles[1]
    if(angles[0]<-180):
        angles[0] = 360+angles[0]
    if(angles[1]<-180):
        angles[1] = 360+angles[1]

    if(obj_pose[0] < 0):
        angles[0] = -angles[0]
    if(obj_pose[1] < 0):
        angles[1] = -angles[1]

    return angles

def get_dvs_from_angles(angles, focl, cam_id):

    px = math.tan((angles[0]*math.pi/180))*focl[0,cam_id-1]
    py = math.tan((angles[1]*math.pi/180))*focl[1,cam_id-1]

    return px, py

def get_optitrack_pose(ip, port):



    cam_poses = set_cam_poses()
    focl = set_focal_lengths()
    
    # Get Rotation Matrices: real-cam to world 
    r2w = get_rotmats(cam_poses)
    
    # Get Translation Matrices: real-cam to world
    r_trl = get_transmats(cam_poses)

    c2w = np.zeros((4,4,3))
    for i in range(3):
        c2w[:,:,i] = r_trl[:,:,i].dot(r2w[:,:,i])

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind((ip, port))  # Replace with the actual origin IP

    out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    while True:
        data, addr = server_socket.recvfrom(1024)
        values = struct.unpack('6d', data)
        for i in range(3): # we forget the angles for now
            xyz_array[i] = round(values[i],3)
        
        ground_truth = [xyz_array[0], xyz_array[1], xyz_array[2], 1]
        perspective = define_object_pose(c2w, ground_truth)
        px_space_array = np.zeros(6)
        for i in range(3):
            angles = get_angles_from_pos(perspective[:,i])
            px_space_array[i*2], px_space_array[i*2+1] = get_dvs_from_angles(angles, focl, i+1)

            message = f"{320+int(px_space_array[i*2])},{240-int(px_space_array[i*2+1])}"
            # print(f"sending {message}")
            out_sock.sendto(message.encode(), ("172.16.222.46", 4331+i))
            



DST_IP = "172.16.222.48"
PORT = 5000

if __name__ == "__main__":

    xyz_array = multiprocessing.Array('d', [0.0,0.0,0.0])
    receiver = multiprocessing.Process(target=get_optitrack_pose, args=(DST_IP, PORT))
    receiver.start()
    receiver.join()