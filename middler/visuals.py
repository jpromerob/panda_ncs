


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np
from scipy import stats
import time
import math




'''
Some function
'''
def plot_gaussians(xyz, pdf, text, scenario):   
    fig = plt.figure(constrained_layout=True, figsize=(15,10))

    N = 5
    gs = GridSpec(3, 10, figure=fig)
    ax_t = fig.add_subplot(gs[:, 0:N])
    # identical to ax1 = plt.subplot(gs.new_subplotspec((0, 0), colspan=3))
    ax_x = fig.add_subplot(gs[0,N:])
    ax_y = fig.add_subplot(gs[1,N:])
    ax_z = fig.add_subplot(gs[2,N:])


    # Text:

    ax_t.text(0.050,0.005,text, family='monospace')
    
    labels = ['Cam 1', 'Cam 2', 'Cam3', 'Prod']
    colors = ['#5b9ad5', '#6fad47', '#febf00', '#000000']
    styles = ['--', '-.', ':' , '-']

    
    for k in range(3+1): # 1,2,3, product
        ax_x.plot(xyz[0,:], pdf[k,0,:], color=colors[k], label=labels[k], linestyle = styles[k])
        ax_y.plot(xyz[1,:], pdf[k,1,:], color=colors[k], label=labels[k], linestyle = styles[k])
        ax_z.plot(xyz[2,:], pdf[k,2,:], color=colors[k], label=labels[k], linestyle = styles[k])


    max_pdf = pdf.max()
        
    ax_t.set_xticks([])
    ax_t.set_yticks([])

    
    ax_x.legend(bbox_to_anchor=(1.2, 0.65))   
    ax_y.legend(bbox_to_anchor=(1.2, 0.65))   
    ax_z.legend(bbox_to_anchor=(1.2, 0.65))   
    
    x_center = xyz[0,np.argmax(pdf[3,0,:])]
    y_center = xyz[1,np.argmax(pdf[3,1,:])]
    z_center = xyz[2,np.argmax(pdf[3,2,:])]
    
    delta = 0.7
    
    min_x = 0.1*int(10*(x_center-delta))
    max_x = 0.1*int(10*(x_center+delta))
    min_y = 0.1*int(10*(y_center-delta))
    max_y = 0.1*int(10*(y_center+delta))
    min_z = 0.1*int(10*(z_center-delta))
    max_z = 0.1*int(10*(z_center+delta))
    
    ax_x.set_xlim([x_center - delta, x_center + delta])
    ax_y.set_xlim([y_center - delta, y_center + delta])
    ax_z.set_xlim([z_center - delta, z_center + delta])
    ax_x.set_ylim([0, max_pdf])
    ax_y.set_ylim([0, max_pdf])
    ax_z.set_ylim([0, max_pdf])

    ax_x.set_xlabel('$x_w$')
    ax_y.set_xlabel('$y_w$')
    ax_z.set_xlabel('$z_w$')

    ax_x.set_xticks(np.arange(min_x, max_x, 0.1))
    ax_y.set_xticks(np.arange(min_y, max_y, 0.1))
    ax_z.set_xticks(np.arange(min_z, max_z, 0.1))

    fig.suptitle("Density of Probability for x, y, z corresponding to cameras 1, 2 and 3")
    
    plt.savefig("tests/scenario_" + str(scenario) + ".png", transparent=False)

    plt.show()



'''
This function estimates how likely it is that a certain (x,y,z) is the true one
mu and sigma are arrays containing mean and std for x,y,z coordinates
The purpose of this function is to faciltiate visualization (not needed for pose merging)
'''
def get_joint(x, y, z, mu, sigma):
    
    p_x = np.exp(-((x-mu[0])**2)/(2*sigma[0]**2))/(sigma[0]*math.sqrt(2*math.pi))
    p_y = np.exp(-((y-mu[1])**2)/(2*sigma[1]**2))/(sigma[1]*math.sqrt(2*math.pi))
    p_z = np.exp(-((z-mu[2])**2)/(2*sigma[2]**2))/(sigma[2]*math.sqrt(2*math.pi))
    
        
    return p_x*p_y*p_z



'''
This function reconstructs a 3D representation of the probabilities of the actual (x,y,z) being somewhere in space
'''
def visualize_3d(nb_pts, mu, sigma):
    
    x = y = z = np.linspace(stats.norm.ppf(0.10), stats.norm.ppf(0.90), nb_pts)

    # Getting joint probabilities
    start = time.time()
    p = np.zeros((nb_pts,nb_pts, nb_pts))
    for idx_x in range (nb_pts): # x, y, z
        for idx_y in range (nb_pts): # x, y, z
            for idx_z in range (nb_pts): # x, y, z   
                p[idx_x, idx_y, idx_z] = get_joint(x[idx_x], y[idx_y], z[idx_z], mu, sigma)

    stop = time.time()
    elapsed = stop - start
    print("Joint probabilities obtained after: " + str(int(elapsed)) + " seconds.")

    # Creating figure
    fig = plt.figure(figsize=(12,12))
    ax = plt.axes(projection="3d")

    idx = p > 5.0

    # Creating plot
    xx, yy, zz = np.meshgrid(x, y, z)
    ax.scatter3D(xx[idx], yy[idx], zz[idx], c=p[idx], cmap='viridis', vmin=0, vmax=10, marker='.')

    xs = np.linspace(-1, 1, nb_pts)
    ys = np.linspace(-1, 1, nb_pts)
    zs = np.linspace(-1, 1, nb_pts)

    X, Y = np.meshgrid(xs, ys)
    Z = np.ones((nb_pts,nb_pts))*mu[2]
    ax.plot_surface(Y, X, Z, alpha=0.2, color='k')

    Y, Z = np.meshgrid(ys, zs)
    X = np.ones((nb_pts,nb_pts))*mu[0]
    ax.plot_surface(Y, X, Z, alpha=0.2, color='k')

    X, Z = np.meshgrid(xs, zs)
    Y = np.ones((nb_pts,nb_pts))*mu[1]
    ax.plot_surface(Y, X, Z, alpha=0.2, color='k')

    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('y')
    ax.set_ylabel('x')
    ax.set_zlabel('z')
    ax.set_xticks(np.linspace(-1, 1, 21))
    ax.set_yticks(np.linspace(-1, 1, 21))
    ax.set_zticks(np.linspace(-1, 1, 21))

    plt.show()  

    