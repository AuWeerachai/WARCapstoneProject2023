## from https://pysource.com/2021/03/11/distance-detection-with-depth-camera-intel-realsense-d435i/#

# modified by Sorie Yillah

import cv2
import pyrealsense2
from realsense_depth import *

# point = (400, 300)
point = (0,0)

def show_distance(event, x, y, args, params):
    global point
    point = (x, y)
    # print(point)
#     if point == (470,283):
#         print(point)

# def print_distance(pt):
#     if pt == (470,283):
#         print("pass")

# Initialize Camera Intel Realsense
dc = DepthCamera()

# Create mouse event
cv2.namedWindow("depth frame")
cv2.setMouseCallback("depth frame", show_distance)

while True:
    ret, depth_frame, colorized_frame, color_frame = dc.get_frame()

    # Show distance for a specific point
    cv2.circle(color_frame, point, 4, (0, 0, 255))
    distance = depth_frame[point[1], point[0]]

    cv2.putText(color_frame, "{}mm".format(distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
    cv2.imshow("depth frame", depth_frame)
    cv2.imshow("Color frame", color_frame)
    cv2.imshow("Color depth", colorized_frame)

    # images = np.hstack((color_frame, colorized_frame, depth_frame))

    # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('RealSense', images)

    key = cv2.waitKey(1)
    # if key == 27 or point == (470,283):
    if key == 27:
        break
