import time
import numpy as np
import cv2
import matplotlib.pyplot as plt

# Read the input image and convert it to grayscale
img = cv2.imread('GetImage.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Apply thresholding on the grayscale image to create a binary image
ret, thresh = cv2.threshold(gray, 150, 255, 0)

# Find the contours in the image
contours, hierarchy= cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

# Select a contour count; Find the Convex Hull for that count
# cnt = contours[0]
# hull = cv2.convexHull(cnt)
for cnt in contours:
    hull = cv2.convexHull(cnt)
    img = cv2.drawContours(img, [cnt], 0, (0,255,0), 2)
    img = cv2.drawContours(img, [hull], 0, (0,0,255), 3)

# Draw the Convex Hull
cv2.drawContours(img, [hull], -1, (0,255,255), 3)

# Display the image with the drawn Convex Hull
# cv2.namedWindow("Convex Hull", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("Convex Hull", 800,800)
# cv2.imshow("Convex Hull", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# Save the image
cv2.imwrite("img_test2.jpg", img)