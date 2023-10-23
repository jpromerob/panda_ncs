
import numpy as np



'''
This function defines ground truth (for a point in space)
'''
def define_object_pose(c2w, ground_truth, delta):
    
    perspective = np.zeros((4,3)) # coordinates|cameras
    # Checking output of each camera
    for i in range(3): # Cam 1, 2, 3
        w2c = np.linalg.inv(c2w[:,:,i])
        perspective[:, i] = w2c.dot(ground_truth) + np.random.uniform(low=-delta, high=delta, size=(4,))

    return perspective[0:3,:]


'''
This function produces 9x2 parameters:mean and std for all combinations of camera/axis (1,2,3 * x,y,z)
Some values are hardcoded in 'e_per' to account for uncertainty for each axis (which depends on camera pose)
'''
def produce_snn_stats(e_per):
       
    
    cam_pdf_params = np.zeros((3,3,2)) # {1,2,3} | {x,y,z} | {mean, std}
    

    #Cam 1
    cam_pdf_params[0,0,:] = [0, e_per[0]] # x
    cam_pdf_params[0,1,:] = [0, e_per[1]] # y
    cam_pdf_params[0,2,:] = [0, e_per[2]] # z

    #Cam 2
    cam_pdf_params[1,0,:] = [0, e_per[0]] # x
    cam_pdf_params[1,1,:] = [0, e_per[1]] # y
    cam_pdf_params[1,2,:] = [0, e_per[2]] # z

    #Cam 3
    cam_pdf_params[2,0,:] = [0, e_per[0]] # x
    cam_pdf_params[2,1,:] = [0, e_per[1]] # y
    cam_pdf_params[2,2,:] = [0, e_per[2]] # z
    
    
    return cam_pdf_params