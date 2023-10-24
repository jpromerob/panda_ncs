import numpy as np

from scipy import stats
from scipy.stats import multivariate_normal
from scipy import stats


''' Create Multivariate Gaussian Distributions'''
def create_mgd(v2r, r2w, trl, μ, Σ, v_obj_poses):   


    r_μ = np.zeros((3,3))
    r_Σ = np.zeros((3,3,3))
    w_μ = np.zeros((3,3))
    w_Σ = np.zeros((3,3,3))
    new_μ = np.zeros((4,3)) # including a '1' at the end
    for k in range(3):
        
                                      
        # Rotating Means from virtual-cam space to real-cam space  
        r_μ[:,k] = v2r[:,:,k] @ μ
                 
        # Rotating Means from real-cam space to world space 
        w_μ[:,k] = r2w[:,:,k] @ r_μ[:,k]
    
        # Translating Means from Camera (Real=Virtual) space to World space 
        new_μ[:,k] = trl[:,:, k] @ [w_μ[0,k], w_μ[1,k], w_μ[2,k],1]                     
                 
        # Rotating Covariance Matrix from virtual-cam space to real-cam space  
        r_Σ[:,:,k] = v2r[:,:,k] @ Σ @ v2r[:,:,k].T  
                 
        # Rotating Covariance Matrix from real-cam space to world space  
        w_Σ[:,:,k] = r2w[:,:,k] @ r_Σ[:,:,k] @ r2w[:,:,k].T 
    
    rv_1 = multivariate_normal(new_μ[0:3,0], w_Σ[:,:,0])
    rv_2 = multivariate_normal(new_μ[0:3,1], w_Σ[:,:,1])
    rv_3 = multivariate_normal(new_μ[0:3,2], w_Σ[:,:,2])
    
    return new_μ, w_Σ, [rv_1, rv_2, rv_3]

def analytical(μ, Σ, presence, old_μ, oldsence):


    mu = np.zeros(3)
    V_n_p = np.zeros((3,3)) 
    
    if presence[0] == 1:
        V_1 = np.linalg.inv(Σ[:,:,0])
        V_n_p += V_1
        μ_1 = μ[0:3,0]
    else:
        V_1 = np.zeros((3,3)) 
        μ_1 = np.zeros(3)

    if presence[1] == 1:
        V_2 = np.linalg.inv(Σ[:,:,1])
        V_n_p += V_2
        μ_2 = μ[0:3,1]
    else:
        V_2 = np.zeros((3,3)) 
        μ_2 = np.zeros(3)

    if presence[2] == 1:
        V_3 = np.linalg.inv(Σ[:,:,2])
        V_n_p += V_3
        μ_3 = μ[0:3,2]
    else:
        V_3 = np.zeros((3,3)) 
        μ_3 = np.zeros(3)

    if np.sum(presence)>=2:
        V_n =np.linalg.inv(V_n_p)
        mu = ((V_1 @ μ_1) + (V_2 @ μ_2) + (V_3 @ μ_3)) @ V_n 
        if np.sum(presence) != np.sum(oldsence):
            mu[0] = (mu[0] + old_μ[0])/2
            mu[1] = (mu[1] + old_μ[1])/2
            mu[2] = (mu[2] + old_μ[2])/2
    else:
        mu[0] = old_μ[0]
        mu[1] = old_μ[1]
        mu[2] = old_μ[2]

    max_delta = 0.002
    for k in range(3): # for x, y, z
        if mu[k] > old_μ[k] + max_delta:
            # print("Jump avoided (up)\n")
            mu[k] = old_μ[k] + max_delta
        if mu[k] < old_μ[k] - max_delta:
            # print("Jump avoided (down)\n")
            mu[k] = old_μ[k] - max_delta


    return mu