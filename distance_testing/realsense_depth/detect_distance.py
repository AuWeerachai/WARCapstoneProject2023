'''
@author: Sorie Yillah

- All citations are in-text
'''

from ArUcoDetector import *
import pyrealsense2 as rs
import time
import cv2
from ArUcoDetector import ArUcoDetector
from realsense_depth import DepthCamera 
import numpy as np
import math
from RoundDetection_withES import FindRounds
from TuningHoughTranform_copyForIntegration import *

cc = DepthCamera()

class GetDistance:
    def __init__(self):
        self.trajectory = dict()

        def clear(self):
            self.trajectory = dict()
        
    def _getCoordinate(self, depth_intrinsics, depth_frame, markerPos):
        # for simplicity, just use the center point to extract the 3D coordinate
        # TODO: In future update, we can use fillPoly to mask the marker, and compute the average 3D coordinate
        topRight, bottomRight, bottomLeft, topLeft = markerPos
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)

        depth = depth_frame.get_distance(cX, cY)
        if depth > 0:
            dist = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [cX, cY], depth)
            # print(dist)
            return dist
        else:
            return None 