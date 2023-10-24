
from scipy import stats
import numpy as np


def data2text(ground_truth, cam_poses, mu_c, sigma_c, mu_w, sigma_w):
    
    text = ""
    text = text + "\n Object:\n"
    text = text + "\n    Pose:"
    text = text + "\n       x = %3.3f | y = %3.3f | z = %3.3f" %(ground_truth[0], ground_truth[1], ground_truth[2])
    for i in range(3):
        text = text + "\n\n Camera %d:\n" %(i+1)
        text = text + "\n    Pose:"
        text = text + "\n       x = %3.3f | y = %3.3f | z = %3.3f" %(cam_poses[i,0], cam_poses[i,1], cam_poses[i,2])
        text = text + "\n       alpha = %3.3f | beta = %3.3f | gamma = %3.3f" %(cam_poses[i,3], cam_poses[i,4], cam_poses[i,5])
        text = text + "\n    PDF:"
        text = text + "\n       Camera Space:"
        text = text + "\n          mu_x = %3.3f | mu_y = %3.3f | mu_z = %3.3f" %(mu_c[0,i], mu_c[1,i], mu_c[2,i])
        text = text + "\n          sigma_x = %3.3f | sigma_y = %3.3f | sigma_z = %3.3f" %(sigma_c[0,i], sigma_c[1,i], sigma_c[2,i])
        text = text + "\n       World Space:"
        text = text + "\n          mu_x = %3.3f | mu_y = %3.3f | mu_z = %3.3f" %(mu_w[0,i], mu_w[1,i], mu_w[2,i])
        text = text + "\n          sigma_x = %3.3f | sigma_y = %3.3f | sigma_z = %3.3f" %(sigma_w[0,i], sigma_w[1,i], sigma_w[2,i])
    
    
    text = text + "\n\n Consolidation:\n"
    text = text + "\n    Pose:"
    text = text + "\n       x_w = %3.3f | y_w = %3.3f | z_w = %3.3f" %(mu_w[0,3], mu_w[1,3], mu_w[2,3])
    
    text = text + "\n\n\n"
    
    e_x_per = 1000*abs(ground_truth[0]-mu_w[0,3])
    e_y_per = 1000*abs(ground_truth[1]-mu_w[1,3])
    e_z_per = 1000*abs(ground_truth[2]-mu_w[2,3])
    print("The estimated object location has the following error in [mm]: (%3.3f, %3.3f, %3.3f)" %(e_x_per, e_y_per, e_z_per))
    
    return text


'''
For each axis, this function produces 3 'basic' PDFs (one per camera) plus 1 'special' PDF (from their product)
The function plots the 4 PDFs (in 3 subplots, one per axis) and returns 'xyz' and 'pdf'
'xyz' is the array of coordinates; 'pdf' is a matrix with the generated PDFs
'''
def generate_pdfs(nb_pts, pdf_center, mu, sigma):
    
    base_xyz = np.linspace(stats.norm.ppf(0.05), stats.norm.ppf(0.95), nb_pts)
    xyz = np.zeros((3,nb_pts))
    pdf = np.zeros((5,3,nb_pts))   
    
    
    # Generating PDFs (+ prodcuts and mixtures)
    for j in range(3): # x, y, z

        
        # This is done to re-center xyz array (for visual purposes)
        xyz[j,:] = base_xyz + pdf_center[j] 

        # Checking output of each camera
        for i in range(3+1): # Cam 1, 2, 3, prod
      
            # PDF is calculated, using stats, with the purpose of visualization (not needed for pose merging)
            pdf[i,j,:] = stats.norm.pdf(xyz[j,:], mu[j,i], sigma[j,i]) 
    
    return xyz, pdf