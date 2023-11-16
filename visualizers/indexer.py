import numpy as np


def find_max_indices(arr):
    # Sum over axis 0
    sum_axis_0 = np.sum(arr, axis=0)
    # Sum over axis 1
    sum_axis_1 = np.sum(arr, axis=1)

    # Get indices where the maximum value occurs for each one-dimensional array
    indices_max_axis_y = np.argmax(sum_axis_0)
    indices_max_axis_x = np.argmax(sum_axis_1)

    return [indices_max_axis_x, indices_max_axis_y]

    
# Assuming your array is named 'arr'
# Replace this with your actual array
arr = np.array([[0,1,0,0,0,0,0,1,0],
                [0,0,0,0,0,0,0,1,0],
                [0,0,0,0,0,0,0,0,0]])

print(arr[0,7])

result = find_max_indices(arr)
print("Indices of the maximum values:", result)
