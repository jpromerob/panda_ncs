import cv2

# Load the lead image from a PNG file
img_lead = cv2.imread('lead_image.png')

# Split the lead image into its color channels
blue_channel, green_channel, red_channel = cv2.split(img_lead)

# Create an empty image with 4 quadrants of size 640x480
img = np.zeros((480*2, 640*2, 3), np.uint8)

# Assign red color to top-left quadrant
img[0:480, 0:640] = (0, 0, 255)

# Assign green color to top-right quadrant
img[0:480, 640:1280] = (0, 255, 0)

# Assign blue color to bottom-left quadrant
img[480:960, 0:640] = (255, 0, 0)

# Assign lead image to bottom-right quadrant
img[480:960, 640:1280] = cv2.merge((blue_channel, green_channel, red_channel))

# Show the final image
cv2.imshow("Image with quadrants", img)
cv2.waitKey(0)
cv2.destroyAllWindows()