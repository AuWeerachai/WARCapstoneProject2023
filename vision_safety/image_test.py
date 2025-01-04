import time
import numpy as np
import cv2
import matplotlib.pyplot as plt

# Read the input image
src = cv2.imread("GetImage.png", 1)
# src = cv2.imread("robot.jpg", 1)

# Binarize the image
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY) # grayscale
blur = cv2.blur(gray, (3, 3)) # blur the image to remove noise
ret, thresh = cv2.threshold(blur, 50, 255, cv2.THRESH_BINARY) # threshhold to binarize image

# Use findContour to find contours for the threshold image
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

# Find the convex Hull
# create hull array for convex hull points 
hull = [] 
# calculate points for each contour 
for i in range(len(contours)): 
    # creating convex hull object for each contour 
    hull.append(cv2.convexHull(contours[i], False)) 

# Draw the Convex Hull
# create an empty black image 
drawing = np.zeros((thresh.shape[0], thresh.shape[1], 3), np.uint8) 
# draw contours and hull points 
for i in range(len(contours)): 
    color_contours = (0, 255, 0) # green - color for contours 
    color = (255, 0, 0) # blue - color for convex hull 
    # draw ith contour 
    cv2.drawContours(drawing, contours, i, color_contours, 1, 8, hierarchy) 
    # draw ith convex hull object 
    cv2.drawContours(drawing, hull, i, color, 1, 8) 

# Display the image with the drawn Convex Hull
# cv2.namedWindow("Convex Hull", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("Convex Hull", 800,800)
# cv2.imshow("Convex Hull", drawing)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# Save the image
cv2.imwrite("img_test.jpg", drawing)