
import numpy as np
import math


'''
This function returns the matrix 'c2w' that converts coordinates in camera space to coordinates in world space.

obj_cam_space = [x_c, y_c, z_c, 1]

c2w = [[rf_11, rf_12, rf_13, rf_14],
       [rf_21, rf_22, rf_23, rf_24],
       [rf_31, rf_32, rf_33, rf_34],
       [rf_41, rf_42, rf_43, rf_44],]

obj_world_space = [x_w, y_w, z_w, 1] = c2w * obj_cam_space

x_w = rf_11*x_c + rf_12*y_c + rf_13*z_c + rf_14*1
y_w = rf_21*x_c + rf_22*y_c + rf_23*z_c + rf_14*1
z_w = rf_31*x_c + rf_32*y_c + rf_33*z_c + rf_14*1

'''
def get_transmats(cam_poses):
    
    c2w = np.zeros((4,4,3))
    for i in range(3): # Cam 1, 2, 3
        
        cx = cam_poses[i,0]
        cy = cam_poses[i,1]
        cz = cam_poses[i,2]
        alpha = cam_poses[i,3]
        beta = cam_poses[i,4] 
        gamma = cam_poses[i,5]

        # Transformation matrices (translation + rotations around x, y, z)
        mat_tran = np.array([[1,0,0,cx],
                             [0,1,0,cy],
                             [0,0,1,cz],
                             [0,0,0,1]])

        mat_rotx = np.array([[1,0,0,0],
                             [0,math.cos(alpha), -math.sin(alpha),0],
                             [0, math.sin(alpha), math.cos(alpha),0],
                             [0,0,0,1]])

        mat_roty = np.array([[math.cos(beta), 0, math.sin(beta),0],
                             [0,1,0,0],
                             [-math.sin(beta), 0, math.cos(beta),0],
                             [0,0,0,1]])


        mat_rotz = np.array([[math.cos(gamma), -math.sin(gamma), 0, 0],
                             [math.sin(gamma), math.cos(gamma),0, 0],
                             [0,0,1,0],
                             [0,0,0,1]])

        # General transformation matrix 'camera to world' (c2w)
        c2w[:,:,i] = mat_tran.dot(mat_rotz).dot(mat_roty).dot(mat_rotx)
    
    
    return c2w


'''
The conflation of a finite number of probability distributions is a consolidation of those distributions into a single probability distribution. If the base distributions are Gaussians, the conflated outcome will be a Gaussian as well. This function estimates the mu and sigma of such gaussian

Inputs: 
 - mu: array of means (from N 'observations' using three instruments: cam 1, 2, 3)
 - sigma: array of stds (from N 'observations' using three instruments: cam 1, 2, 3)
 
 Outputs:
 - mu_conflation: mean of conflation of gaussians
 - sigma_conflation: std of conflation of gaussians
 
'''
def conflate(mu, sigma):
    
    mu_1 = mu[0]
    mu_2 = mu[1]
    mu_3 = mu[2]
    
    ss_1 = sigma[0]**2
    ss_2 = sigma[1]**2
    ss_3 = sigma[2]**2
    
    
    mu_conflation = (ss_1*ss_2*mu_3 + ss_2*ss_3*mu_1 + ss_3*ss_1*mu_2)/(ss_3*ss_2 + ss_2*ss_1 + ss_1*ss_3)
    sigma_conflation = math.sqrt((ss_1 * ss_2 * ss_3)/(ss_1*ss_2 + ss_2*ss_3 + ss_3*ss_1))
    
    
    return mu_conflation, sigma_conflation


'''
This function converts gaussians from camera space to world space
It also calculates the product of the 3 resulting gaussians (1 per camera) per axis in world space
Such product is given by the Gaussian's mean and std

if X ~ N[mu_x, sigma_x^2] then:
    (a*X + b) ~ N[a*mu_x+b, (a*sigma_x)^2]

if X ~ N[mu_x, sigma_x^2] and Y ~ N[mu_y, sigma_y^2] and Z ~N[mu_z, sigma_z^2] are independent, then:
    X+Y+Z ~ N[mu_x+mu_y+mu_z, sigma_x^2+sigma_y^2+sigma_z^2]

x_w ~ N[rf_11*mu_x_c + rf_12*mu_y_c + rf_13*mu_z_c + rf_14, (rf_11*sigma_x_c)^2 +(rf_12*sigma_y_c)^2 + (rf_13*sigma_z_c)^2]
y_w ~ N[rf_21*mu_x_c + rf_22*mu_y_c + rf_23*mu_z_c + rf_24, (rf_21*sigma_x_c)^2 +(rf_22*sigma_y_c)^2 + (rf_23*sigma_z_c)^2]
z_w ~ N[rf_31*mu_x_c + rf_32*mu_y_c + rf_33*mu_z_c + rf_34, (rf_31*sigma_x_c)^2 +(rf_32*sigma_y_c)^2 + (rf_33*sigma_z_c)^2]

'''  
def get_world_gaussian(perspective, cam_pdf_params, c2w):
    
    mu_c = np.zeros((3,4))
    sigma_c = np.zeros((3,4))
    mu_w = np.zeros((3,4))
    sigma_w = np.zeros((3,4))
        
    # The gaussians are centered around the values given by the SNN (in camera space: 'perspective')
    for j in range(3): # x, y, z
        for i in range(3): # Cam 1, 2, 3
            mu_c[j,i] = perspective[j,i] + cam_pdf_params[i,j,0]
            sigma_c[j,i] = cam_pdf_params[i,j,1]    

    # The gaussians are transformed to world space and their product is calculated (once per axis)
    for j in range(3): # x, y, z
        for i in range(3): # Cam 1, 2, 3
            mu_w[j, i] = c2w[j,0,i]*mu_c[0,i] + c2w[j,1,i]*mu_c[1,i] + c2w[j,2,i]*mu_c[2,i] + c2w[j,3,i]
            sigma_w[j, i] = math.sqrt((c2w[j,0,i]*sigma_c[0,i])**2 +(c2w[j,1,i]*sigma_c[1,i])**2 + (c2w[j,2,i]*sigma_c[2,i])**2)
        
        mu_w[j, 3], sigma_w[j, 3] = conflate(mu_w[j, 0:3], sigma_w[j, 0:3])
    
    return mu_c, sigma_c, mu_w, sigma_w