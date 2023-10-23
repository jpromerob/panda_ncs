
import numpy as np
import cv2
 
logo = cv2.imread('NCS_640_480.png')
print(logo.shape)
image = 125*np.ones((480+200*2, 640+200*2,3))
image[:,:,0] = 255
image[:,:,1] = 200
image[:,:,2] = 23
image[100:580, 100:740,:]=logo
 
cv2.imshow('sample image',image)
 
cv2.waitKey(0) # waits until a key is pressed
cv2.destroyAllWindows()
