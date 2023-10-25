import cv2
import numpy as np

# Create a numpy array A
A = np.zeros((1280, 960, 3), dtype=np.uint8)

# Assign values to the four quadrants
A[0:640, 0:480, 0] = np.ones((640, 480))
A[640:1280, 0:480, 1] = np.ones((640, 480))
A[0:640, 480:960, 2] = np.ones((640, 480))

# Load the PNG image
image = cv2.imread('ncs_logo_640_480.jpg')  # Replace 'your_image.png' with the path to your PNG image

# Resize the image to match the quadrant dimensions (640x480)
# resized_image = cv2.resize(image, (480, 640))  # Swap width and height to match the dimensions of the quadrant

# Superimpose the resized image onto the specified quadrant in A
A[640:1280, 480:960, :] = image.transpose(1,0,2)

# Display or save the resulting image if needed
cv2.imshow('Superimposed Image', A)
cv2.waitKey(0)
cv2.destroyAllWindows()

# If you want to save the resulting image
# cv2.imwrite('result.png', A)