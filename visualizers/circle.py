import numpy as np
import matplotlib.pyplot as plt

def create_circle(radius):
    # Create a 2D array filled with zeros
    side_length = 2 * radius + 1
    circle = np.zeros((side_length, side_length))

    # Create a meshgrid of indices
    x, y = np.indices((side_length, side_length))

    # Calculate the center of the circle
    center = (radius, radius)

    # Set values in the circle to 1 (or any desired non-zero value)
    mask = (x - center[0])**2 + (y - center[1])**2 <= (radius)**2
    circle[mask] = 1

    return circle

# Create the circle array using radius
circle_array = create_circle(20)

# Display the result using matplotlib
plt.imshow(circle_array, cmap='gray', interpolation='nearest')
plt.show()
