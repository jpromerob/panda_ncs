import numpy as np
import math


'''
This function sets the camera poses based on manual readings from optitrack (using camera marker 'hat')
'''
def set_cam_poses():

    cam_poses = np.zeros((3,6))

    # Cam 1
    cam_poses[0,0] = -0.135 # cam1:cx
    cam_poses[0,1] = 0.923 # cam1:cy
    cam_poses[0,2] = 1.404 # cam1:cz
    cam_poses[0,3] = (math.pi/180)*(-71.206) # cam1:alpha
    cam_poses[0,4] = (math.pi/180)*(11.003) # cam1:beta
    cam_poses[0,5] = (math.pi/180)*(-23.995) # cam1:gamma

    # Cam 2
    cam_poses[1,0] = -0.652 # cam2:cx
    cam_poses[1,1] = 0.931 # cam2:cy
    cam_poses[1,2] = 1.420 # cam2:cz
    cam_poses[1,3] = (math.pi/180)*(-49.949) # cam2:alpha
    cam_poses[1,4] = (math.pi/180)*(-46.840) # cam2:beta
    cam_poses[1,5] = (math.pi/180)*(-12.425) # cam2:gamma

    # Cam 3
    cam_poses[2,0] = -0.720 # cam3:cx
    cam_poses[2,1] = 0.918 # cam3:cy
    cam_poses[2,2] = 0.553 # cam3:cz
    cam_poses[2,3] = (math.pi/180)*(167.364) # cam3:alpha
    cam_poses[2,4] = (math.pi/180)*(-57.342) # cam3:beta
    cam_poses[2,5] = (math.pi/180)*(125.524) # cam3:gamma

    return cam_poses

'''
Set 'focal lengths' (from cameras' intrinsic matrices)
'''
def set_focal_lengths():

    focl = np.zeros((2,3))

    focl[0,0] = 657.695274 
    focl[0,1] = 695.410317
    focl[0,2] = 775.276734
    focl[1,0] = 656.424014 
    focl[1,1] = 690.799429
    focl[1,2] = 774.173048

    return focl

'''
Set 'principal point coordinates' (from cameras' intrinsic matrices)
'''
def set_pp_coordinates():

    pp_coor = np.zeros((2,3))

    pp_coor[0,0] = 334.641334 - 0
    pp_coor[0,1] = 323.043462 - 5
    pp_coor[0,2] = 310.350021 - 5
    pp_coor[1,0] = 256.919557 + 10
    pp_coor[1,1] = 227.992238 - 40
    pp_coor[1,2] = 250.150330 + 0

    return pp_coor

'''
Poses of virtual cameras with respect to their corresponding real camera space
'''
def set_vir_poses(angles, v_poses, presence):
   
    for k in range(3): # for cameras 1|2|3
        # Cam k+1
        if presence[k]==1:
            v_poses[k,0] = 0
            v_poses[k,1] = 0
            v_poses[k,2] = 0
            v_poses[k,3] = (math.pi/180)*(angles[1,k])    # around X axis -->  ang(YZ)
            v_poses[k,4] = (math.pi/180)*(-angles[0,k])   # around Y axis --> -ang(XZ)
            v_poses[k,5] = 0 

    return v_poses
    

''' Translation Matrices'''
def get_transmats(viewer_poses):
    
    nb_viewers = viewer_poses.shape[0]

    mat_tran = np.zeros((4,4,nb_viewers))
    for i in range(nb_viewers): # Cam 1, 2, 3
        
        cx = viewer_poses[i,0]
        cy = viewer_poses[i,1]
        cz = viewer_poses[i,2]

        # Transformation matrices (translation + rotations around x, y, z)
        mat_tran[:,:,i] = np.array([[1,0,0,cx],
                                    [0,1,0,cy],
                                    [0,0,1,cz],
                                    [0,0,0,1]])
        
    return mat_tran
    
    
'''Rotation Matrices'''
def get_rotmats(viewer_poses):

    nb_viewers = viewer_poses.shape[0]
    
    mat_rota = np.zeros((4,4,nb_viewers))
    for i in range(nb_viewers): # Cam 1, 2, 3
        
        alpha = viewer_poses[i,3]
        beta = viewer_poses[i,4] 
        gamma = viewer_poses[i,5]


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
This function returns object pose in three perspectives (one per camera)
'''
def get_perspectives(c2w, ground_truth):
    
    # Number of cameras (viewers)
    nb_viewers = c2w.shape[2]

    perspective = np.zeros((4,nb_viewers)) # coordinates|cameras
    # Checking output of each camera
    for i in range(nb_viewers): # Cam 1, 2, 3
        w2c = np.linalg.inv(c2w[:,:,i])
        perspective[:, i] = w2c.dot(ground_truth)

    return perspective[0:3,:]


'''
This functions determines the angular 'distance' between camera and object in planez XZ and YZ
'''
def get_angles_from_dvs(px, py, focl, cam_id):

    angles = np.zeros(2)
    
    angles[0] = (180/math.pi)*math.atan2(px, focl[0,cam_id-1]) 
    angles[1] = (180/math.pi)*math.atan2(py, focl[1,cam_id-1]) 

    return angles

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

def get_dvs_from_angles(angles, focl, pp_coor, cam_id):

    px = pp_coor[0,cam_id-1]+int(math.tan((angles[0]*math.pi/180))*focl[0,cam_id-1])
    py = 480-(pp_coor[1,cam_id-1]-int(math.tan((angles[1]*math.pi/180))*focl[1,cam_id-1]))


    return px, py
