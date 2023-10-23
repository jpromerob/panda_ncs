import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# Define object dimensions
w = 60/1000   # width
h = 40/1000   # height
d = 25/1000   # depth

# Define object location
x = -664/1000   # center x-coordinate
y = 979/1000   # center y-coordinate
z = 538/1000   # center z-coordinate

# Define rotation angles in degrees
rot_x = 148.698
rot_y = -46.056
rot_z = 148.752

# Convert rotation angles to radians
rot_x = math.radians(rot_x)
rot_y = math.radians(rot_y)
rot_z = math.radians(rot_z)

# Define rotation matrices
R_x = np.array([[1, 0, 0, 0], 
               [0, math.cos(rot_x), -math.sin(rot_x), 0], 
               [0, math.sin(rot_x), math.cos(rot_x), 0], 
               [0, 0, 0, 1]])

R_y = np.array([[math.cos(rot_y), 0, math.sin(rot_y), 0], 
               [0, 1, 0, 0], 
               [-math.sin(rot_y), 0, math.cos(rot_y), 0], 
               [0, 0, 0, 1]])

R_z = np.array([[math.cos(rot_z), -math.sin(rot_z), 0, 0], 
               [math.sin(rot_z), math.cos(rot_z), 0, 0], 
               [0, 0, 1, 0],
               [0, 0, 0, 1]])

# Define translation matrix
T = np.array([[1, 0, 0, x], 
             [0, 1, 0, y], 
             [0, 0, 1, z],
             [0, 0, 0, 1]])

# Compute transformation matrix
transformation_matrix = T.dot(R_x).dot(R_y).dot(R_z)

# Define box vertices
vertices = np.array([[-w/2, -h/2, -d/2, 1],
                     [w/2, -h/2, -d/2, 1],
                     [w/2, h/2, -d/2, 1],
                     [-w/2, h/2, -d/2, 1],
                     [-w/2, -h/2, d/2, 1],
                     [w/2, -h/2, d/2, 1],
                     [w/2, h/2, d/2, 1],
                     [-w/2, h/2, d/2, 1]])

# Apply transformation to vertices
new_vertices = transformation_matrix.dot(vertices.T).T[:,:3]

# Define plot limits
x_limits = (-0.8, 0.8)
y_limits = (0, 1.2)
z_limits = (0, 1.6)

# Create plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot box
ax.scatter(new_vertices[:,0], new_vertices[:,1], new_vertices[:,2])
ax.set_xlim(x_limits)
ax.set_ylim(y_limits)
ax.set_zlim(z_limits)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_ylabel('Z')

plt.show()
