## Detector for ArUco Markers with Intel RealSense Camera
## Author: zptang (UMass Amherst)



# old_circles = []
# param = [30,150,17,12,15]
# ES_flag = 2

class RealsenseTracker:
    def __init__(self) -> None:
        self.coordinates = Coordinates()

    def main(self):
        old_circles = []
        param = [30,150,17,12,15]
        ES_flag = 2
        minRadius = 0.025
        maxRadius = 0.035
        # x = 0
        # y = 0

        dict_to_use, visualize, grey_color, _ = readConfig('./config.json')

        arucoDetector = ArUcoDetector(dict_to_use)
        tracker = TrajectoryTracker()

        camera = Camera()
        camera.startStreaming()

        # erodeKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5)) # https://stackoverflow.com/questions/52677479/exact-depth-distance-from-realsense-d435-with-x-y-coordinates

        # fgbg = cv2.createBackgroundSubtractorMOG2() # https://stackoverflow.com/questions/52677479/exact-depth-distance-from-realsense-d435-with-x-y-coordinates
        
        if visualize: start_time = time.time()
        try:
            # while True:
            for _ in range(10):
                frame = camera.getNextFrame()
                depth_image, color_image, colorized_image = camera.extractImagesFromFrame(frame)

                # Remove unaligned part of the color_image to grey
                depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
                masked_color_image = np.where(depth_image_3d <= 0, grey_color, color_image)

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                # Detect markers and draw them on the images
                result = arucoDetector.detect(color_image)
                color_image_output = ArUcoDetector.getImageWithMarkers(color_image, result)
                if color_image_output:
                    color_image = color_image_output[0]
                masked_color_image_output = ArUcoDetector.getImageWithMarkers(masked_color_image, result)
                if masked_color_image_output:
                    masked_color_image = masked_color_image_output[0]
                depth_colormap_output = ArUcoDetector.getImageWithMarkers(depth_colormap, result)
                if depth_colormap_output:
                    depth_colormap = depth_colormap_output[0]

                # Inserted --------------------------------------------------------------------

                try:
                    # print(f'ES_flag = {ES_flag}')
                    if ES_flag == 0:
                        print("testing")
                        ES = EvolutionStrategy(3,15,fitness,[100,50,30,10,150], sigma = 15, frame = color_image)
                        
                        high_gradient, low_gradient, accumulator_threshold, minRadius, maxRadius = ES.optimize()
                        ES_flag = 1
                        #cycle to the next frame for processing        
                        frame = camera.getNextFrame()
                        depth_image, color_image = camera.extractImagesFromFrame(frame)
                    elif ES_flag == 3:
                        print("here")
                        param = list(map(int, input("Type parameter numbers with space: ").split()))
                        high_gradient, low_gradient, accumulator_threshold, minRadius, maxRadius = param
                        print("Tune Again?")
                        ES_flag = input("If done enter 1. If not, enter 3")
                    else:
    #                    param = [50,100,20,3,25]
                        high_gradient, low_gradient, accumulator_threshold, minRadius, maxRadius = param

                except:
                    if ES_flag == 0:
                        print("test fail")
                    else: 
                        print("look at lines 172 to 186")


                rounds = FindRounds(color_image, old_circles, high_gradient, low_gradient, accumulator_threshold, minRadius, maxRadius, self.coordinates)
                output = rounds.output()

                color_image = output[0]
                circles = output[1]
                old_circles = output[2]
                printables = output[3]
                reset_num = output[4]
                edge_img = output[5]
                # pixelPoints = output[6]
                # pixel_point = output[6]

                #---------------------------------------------------------------------------------
                # Pose Estimation
                depth_frame = frame.get_depth_frame()
                depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
                # Finding the Rotation Matrix for of the Camera wrt the world
                f_x= depth_intrinsics.fx
                f_y= depth_intrinsics.fy
                c_x= depth_intrinsics.ppx
                c_y = depth_intrinsics.ppy
                camera_matrix = np.array([[f_x,0,c_x],
                                        [0,f_y,c_y],
                                        [0 ,0 ,1 ]])
                            
                            
                dist_coeff= np.array([[depth_intrinsics.coeffs[0],depth_intrinsics.coeffs[1],depth_intrinsics.coeffs[2],depth_intrinsics.coeffs[3],depth_intrinsics.coeffs[4]]])
            
                if color_image_output:
                    ret = cv2.aruco.estimatePoseSingleMarkers(color_image_output[1], 0.05, camera_matrix, dist_coeff)
                    rvec = ret[0][0, 0, :]
                    tvec = ret[1][0, 0, :]
                    cv2.drawFrameAxes(color_image, camera_matrix,dist_coeff,rvec,tvec,0.05)

                # marked id
                # cv2.drawDetectedMarkers(color_image, mc, mid)

                #---------------------------------------------------------------------------------
                # Coordinates 

                self.coordinates.transform(depth_frame, depth_intrinsics)

                # newL = []
                # finalL = []
                # # https://www.geeksforgeeks.org/read-content-from-one-file-and-write-it-into-another-file/
                # with open('pixelPoint.txt', 'r') as input:
                #     with open('coordinateList.txt', 'w') as output:
                #         with open('coordinate.txt', 'w') as output2:
                #             # for line in input:
                #             # https://stackoverflow.com/questions/3142054/python-add-items-from-txt-file-into-a-list
                #                 # pixelList = input.readlines()
                #             pixelList = [line.strip() for line in input]
                #             # pixelList = list(set(pixelList))
                #             # pixelList = [int(i) for i in pixelList]
                #             output.write(str(pixelList))
                #             # for i in pixelList:
                #             #     output2.write(i+"\n")
                #             # tupleList = [tuple(i) for i in pixelList]
                #             tupleList = list(map(eval, pixelList))
                #             for i in tupleList:
                #                 depth = depth_frame.get_distance(int(i[0]), int(i[1]))
                #                 if depth < 0.99 and depth > 0.80:
                #                 # if depth > 0:
                #                     dist = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [int(i[0]), int(i[1])], depth)
                #                     # distance.append(dist)
                #                 output2.write(str(dist) + "\n")

                # Convert from camera space to robot coordinate frame
                # with open('robot.txt', 'w') as output3:
                #     with open('coordinate.txt','r') as read_file:
                #         for line in read_file:
                #             line = line[1:-2]
                #             line = line.split(", ")
                #             for i in line:
                #                 newL.append(float(i))
                #             finalL.append(newL)
                #             newL = []

                #         for i in finalL:
                #             output3.write(f'({0.20 - i[1]}, {-i[0]})'+'\n')
                
                # Use tracker to find center of the Aruco tag
                # tracker._getCoordinate(depth_intrinsics, depth_frame, mc)
                # if color_image_output:
                #     cx = int((color_image_output[1][0][3][0]+color_image_output[1][0][1][0])/2.0)
                #     cy = int((color_image_output[1][0][3][1]+color_image_output[1][0][1][1])/2.0)
                #     newDepth = depth_frame.get_distance(cx, cy)
                #     if newDepth > 0:
                #         newDist = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [cx, cy], newDepth)
                #         # newDist[0] = newDist[0] + 0.03
                #     if os.path.exists("markerCoord.txt"):
                #         os.remove("markerCoord.txt")
                #     else:
                #         print("The file doesn't exist")
                #     outfile = open("markerCoord.txt", 'w')
                #     outfile.write(str(newDist))

                # find the distance from the Aruco tag to each point


                #---------------------------------------------------------------------------------
                robot = color_image
                # cv2.imwrite("robot.jpg", robot)
                gray = cv2.cvtColor(robot, cv2.COLOR_BGR2GRAY)
                ret, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)

                # Detect the rounds
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
                    
                    
                # cv2.namedWindow("Convex Hull", cv2.WINDOW_AUTOSIZE)
                # cv2.resizeWindow("Convex Hull", 1960,1080)
                # cv2.imshow("Convex Hull", drawing)
                # cv2.waitKey(1)

                # ---------------------------------------------------------------------------------
                # Update trajectory
                tracker.updateTrajectory(frame, result)

                # Show images
                # depth_colormap = np.dstack((depth_colormap, depth_colormap, depth_colormap)) #depth image is 1 channel, color is 3 channels
                # print(type(color_image))
                
                # Moved up by Dan and Sorie

                # if visualize:
                #     current_time = time.time()
                #     # print(current_time-start_time)
                #     if current_time - start_time >= 20:
                #         tracker.plotTrajectory()
                #         # tracker.clear()
                #         start_time = current_time

                # Additional frame by Sorie from old code
                edge_img = np.dstack((edge_img,edge_img,edge_img)) 
                # depth_image = np.dstack((depth_image, depth_image, depth_image))

                # The plot image
                # tracker.plotTrajectory()
                # image = cv2.imread("img.png")
                # image = np.dstack((image, image, image))

                images = np.hstack((color_image, masked_color_image, edge_img, depth_colormap))
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', images)
                # # cv2.imshow('Realsense', color_image)
                # # cv2.imshow('depth', colorized_image)
                # # cv2.imshow("plot", image)
                cv2.waitKey(1)
            return self.coordinates

        finally:
            camera.stopStreaming()

if __name__ == '__main__':
    import time
    import numpy as np
    import cv2
    from ArUcoDetector import ArUcoDetector
    from Camera import Camera
    from TrajectoryTracker import TrajectoryTracker
    from ArUcoGenerator import readConfig
    from RoundDetection_withES import *
    # import transform
    realsense_tracker = RealsenseTracker()
    realsense_tracker.main()
    # transform.main()
else:
    # relative imports
    import time
    import numpy as np
    import cv2
    from .ArUcoDetector import ArUcoDetector
    from .Camera import Camera
    from .TrajectoryTracker import TrajectoryTracker
    from .ArUcoGenerator import readConfig
    from .RoundDetection_withES import *