from _6DOF_test import invKJustPos
# from _6DOF_final import Calculations_MultiplePoints
# from tracking_testing import RealsenseTracker as rt
import numpy as np
import matplotlib.cm as cm
from tracking_testing.Camera import Camera
from tracking_testing.ArUcoDetector import ArUcoDetector
from tracking_testing.RoundDetection_withES import FindRounds
from tracking_testing.TrajectoryTracker import TrajectoryTracker
from tracking_testing.TuningHoughTranform_copyForIntegration import *
from positions.positions import Coordinates
from _6DOF_final.RunCalculations_Class import RobotArm
from _2DOF_test.parallel_processing.torque_sense_import import TorqueProducer
import multiprocessing as mp
from time import time, sleep
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure
from customtkinter import CTkImage, CTkLabel
from PIL import Image
from collections import deque
import pyrealsense2 as rs

def exit(window, event):
    '''
    Closes the window and exits the application
    '''
    if event:
        print(f'[controller] exit: {event.state}-{event.keysym}')
    window.destroy()

def start(window):
    '''
    Starts the main loop of the robot arm detecting and moving
    '''
    # robot = RobotArm(window)
    window.robot.demo_path()
    # 
    # phase 1: find the rounds and Aruco and copy coordinates to a variable (somewhere in a shared class)
    # phase 2: turn off aruco and clear list so robot can move a round (get camera ready for vision safety)
    # phase 3: calculate arm movement trajectory based on round coordinates and move
    # phase 4: after robot is done moving, populate list with new locations to check accuracy
        

def phase1(window):
    # robot = RobotArm(window)
    # robot.execute_path([
    #     np.matrix([[0], [0.54], [0.1]]),
    #     np.matrix([[0.50], [0.54], [0.1]]),
    #     np.matrix([[0.50], [-0.60], [0.1]]),
    #     np.matrix([[0], [-0.60], [0.1]])
    # ])
    pass

def phase2():
    pass

def phase3():
    pass

def phase4():
    pass

def stop():
    '''
    Stops the main loop of the robot
    '''
    pass

def slider_scroll(event, widget, speed):
    var = widget.cget('variable')
    current = var.get()
    if event.delta > 0 and current < widget.cget('to'):
        var.set(current + speed)
    elif event.delta < 0 and current > widget.cget('from_'):
        var.set(current - speed)

def plot(master, func):
    fig = Figure(figsize=(10, 10), dpi=100)
    canvas = FigureCanvasTkAgg(figure=fig, master=master)
    canvas.draw()
    func(fig)
    canvas.get_tk_widget().pack()
    toolbar = NavigationToolbar2Tk(canvas, master)
    toolbar.update()
    canvas.get_tk_widget().pack()

def simulate_path(fig):
    Tol = 0.05
    NumberP = 5
    Startangles = np.matrix([[3.54],[2.33],[-4.49],[-1.94],[-0.44],[-2.24]])
    desiredXYZ = np.matrix([[0], [0.5], [0.5]])
    pos = invKJustPos.StraightLine(Startangles, desiredXYZ,Tol,NumberP)
    ax = fig.add_subplot(111, projection='3d')
    x, y, z = invKJustPos.toXYZforPlot(pos)
    ax.grid()
    # alphas = np.linspace(0.2, 1, np.shape(x)[1])
    colors = cm.rainbow(np.linspace(0, 1, np.shape(x)[1]))
    ax.scatter(x[0], y[0], z[0], c='r', s=35, marker='1')
    ax.scatter(0, 0, 0, c = 'black', s = 35)
    ax.scatter(x, y, z, c = colors, s = 25, alpha = 1)
    ax.scatter(desiredXYZ[0,0], desiredXYZ[1,0], desiredXYZ[2,0], c = 'g', s = 35, marker = "1")
    ax.set_title('Path of Arm')
    # Set axes label
    ax.set_xlabel('x', labelpad=20)
    ax.set_ylabel('y', labelpad=20)
    ax.set_zlabel('z', labelpad=20)

class TorquePlot:
    def __init__(self, master, running):
        self.master = master
        self.running = running
        self.queue = mp.Queue()
        self.stop_movement_event = mp.Event()
        self.stop = mp.Event()
        self.torque_produce = mp.Process(target=TorqueProducer, args=(self.queue, self.stop_movement_event, self.stop), daemon=True)
        maximum_length = 200
        self.points = [deque(maxlen=maximum_length) for _ in range(8)]

    def toggle(self):
        if self.running.get():
            fig = Figure(figsize=(5, 5), dpi=100)
            self.canvas = FigureCanvasTkAgg(figure=fig, master=self.master)
            self.canvas.get_tk_widget().pack()
            self.toolbar = NavigationToolbar2Tk(self.canvas, self.master)
            self.toolbar.update()
            print(self.torque_produce.is_alive())
            if not self.torque_produce.is_alive():
                self.start_time = time()
                self.torque_produce.start()
            self.stop.clear()
            self.next_frame(fig)
        else:
            self.stop.set()
            print("torque graph exited")
    
    def next_frame(self, fig):
        if not self.stop.is_set():
            while not self.queue.empty():
                torque_queue_value = self.queue.get()
                currentTime = time() - self.start_time
                # print(torque_queue_value)
                self.points[0].append(currentTime)
                self.points[1].append(torque_queue_value[0])
                self.points[2].append(torque_queue_value[1])
                self.points[3].append(torque_queue_value[2])  
                self.points[4].append(torque_queue_value[3])
                self.points[5].append(torque_queue_value[4])
                self.points[6].append(torque_queue_value[5])
                self.points[7].append(torque_queue_value[6])
            ax = fig.add_subplot(111)
            p1, = ax.plot(self.points[0], self.points[1], label="m1")
            p2, = ax.plot(self.points[0], self.points[2], label="m2")
            p3, = ax.plot(self.points[0], self.points[3], label="m3")
            p4, = ax.plot(self.points[0], self.points[4], label="m5")
            p5, = ax.plot(self.points[0], self.points[5], label="m6")
            p6, = ax.plot(self.points[0], self.points[6], label="m7")
            p7, = ax.plot(self.points[0], self.points[7], label="m8")
            ax.set_xlabel('Time [s]')
            ax.set_ylabel('Torque [N-m]')
            ax.legend([p1, p2, p3, p4, p5, p6, p7,],["m1","m2","m3", "m5", "m6", "m7", "m8"], loc = 2)
            self.canvas.draw()
            # self.toolbar.update()
            # self.canvas.get_tk_widget().pack()
            fig.clf()

            self.master.after(10, lambda:self.next_frame(fig))

class VideoFeed:
    def __init__(self, master, running, coordinates, output, depth_var, aruco_var, circle_var, gamma, depth_slider):
        self.master = master
        self.running = running
        self.coordinates = coordinates
        self.text = output
        self.depth_overlay_var = depth_var
        self.aruco_overlay_var = aruco_var
        self.circle_overlay_var = circle_var
        self.gamma_var = gamma
        self.depth_slider_var = depth_slider
        self.video = CTkLabel(master, text="")
        self.camera_queue = mp.Queue()
        self.coordinate_queue = mp.Queue()
        self.camera_stop = mp.Event()
        self.camera = mp.Process(target=DepthCamera, args=(self.camera_queue, self.coordinate_queue, self.camera_stop), daemon=True)
        self.camera_stop.set()
        self.camera.start()

    def toggle(self, idle_image):
        if self.running.get():
            # print(self.camera.is_alive())
            # if not self.camera.is_alive():
            #     self.camera.start()
            self.camera_stop.clear()
            self.next_frame(idle_image)
        else:
            self.camera_stop.set()
            self.video.pack_forget()
            idle_image.place(relx=0.5, rely=0.5, anchor="center")
    
    def next_frame(self, idle_image=None):
        # pull images from shared queue
        camera_consume_rate = 1
        coord_consume_rate = 1
        # if there's too much backlog on the queue, we pull from it faster until they run smoothly
        if (self.camera_queue.qsize() > 6):
            camera_consume_rate = 5
        if (self.coordinate_queue.qsize() > 6):
            coord_consume_rate = 5
        if not self.camera_stop.is_set(): 
            for _ in range(camera_consume_rate):
                # get images from camera
                aruco, color_image, depth_colormap, circles = self.camera_queue.get()
            # get coordinates from camera
            self.coordinates.clear()
            self.text.configure(state='normal')
            self.text.delete('0.0', 'end')
            for _ in range(coord_consume_rate):
                coords = self.coordinate_queue.get()
            for point in coords:
                point = point.split(':')
                key = int(point[0])
                value = point[1].strip('()').split(', ')
                x = self.master.master.camera_offset_x.get() - (float(value[0]) * self.master.master.camera_scale.get())
                y = (float(value[1]) * self.master.master.camera_scale.get()) - self.master.master.camera_offset_y.get()
                self.text.insert(f'{key}.0', f'{key}: ({x:.2f}, {y:.2f})\n')
                self.coordinates.add_coordinate(key, (x, y))
            self.text.configure(state='disabled')
   
            # Build Overlays
            img = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            img = cv2.addWeighted(img, 1, img, 0, self.gamma_var.get())
            if self.depth_overlay_var.get():
                img = cv2.addWeighted(cv2.cvtColor(depth_colormap, cv2.COLOR_BGR2RGB), self.depth_slider_var.get(), img, 1-self.depth_slider_var.get(), 0)
            if self.aruco_overlay_var.get() and aruco:
                    # draw aruco tag
                for item in aruco:
                    topRight, bottomRight, bottomLeft, topLeft, cX, cY, markerID, camera_matrix, dist_coeff, rvec, tvec = item
                    cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)
                    cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
                    cv2.putText(img, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.drawFrameAxes(img, camera_matrix, dist_coeff, rvec, tvec, 0.05)
            if self.circle_overlay_var.get() and circles is not None:
                for i in circles:
                    # color is BGR
                    color1 = (0,255,0)
                    if i[2] <= 14:
                        cv2.circle(img, (i[0], i[1]), i[2], color1, 2) #last var is thickness
                    
                    #draw the center with a small circle
                    color2 = (255,0,0)
                    if i[2] <= 14:
                        cv2.circle(img, (i[0], i[1]), 2, color2, 3)
                        if self.depth_overlay_var.get():
                            cv2.putText(img, str((i[0], i[1])), (i[0], i[1]), 100, 1, (255, 255, 255))
                        else:
                            cv2.putText(img, str((i[0], i[1])), (i[0], i[1]), 100, 1, (0, 0, 255))

            # Display Image
            image = CTkImage(Image.fromarray(img), size=(self.master.winfo_width(), self.master.winfo_height()))
            self.video.configure(image=image)

            # pack widget
            if idle_image is not None:
                idle_image.place_forget()
                self.video.pack(expand=True, padx=4, pady=4)
            self.master.after(10, self.next_frame)

class DepthCamera(Camera):
    def __init__(self, video_queue, point_queue, event):
        super().__init__()
        self.vq = video_queue
        self.pq = point_queue
        self.event = event
        self.old_circles = []
        self.param = [30,150,17,12,15]
        self.ES_flag = 2
        self.minRadius = 0.025
        self.maxRadius = 0.035
        self.aruco = ArUcoDetector("DICT_4X4_250")
        # self.aruco_2 = ArUcoDetector("DICT_4X4_250")
        self.tracker = TrajectoryTracker()
        self.coordinates = Coordinates()
        self.startStreaming()
        while True:
            if not event.is_set():
                self.update()
            sleep(0.01)

    def update(self):
        frame = self.getNextFrame()
        if frame is not None:
            depth_image, color_image, colorized_image = self.extractImagesFromFrame(frame)

            # Remove unaligned part of the color_image to grey
            grey_color = 153
            depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
            masked_color_image = np.where(depth_image_3d <= 0, grey_color, color_image)

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # Detect markers and draw them on the images
            result = self.aruco.detect(color_image)
            aruco_image = self.aruco.getImageWithMarkers(color_image, result, True)
            # result_2 = self.aruco_2.detect(color_image)
            # aruco_image_2 = self.aruco_2.getImageWithMarkers(color_image, result_2, True)

            # Pose Estimation
            depth_frame = frame.get_depth_frame()
            depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            color_frame = frame.get_color_frame()
            color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

            # circle detection
            try:
                # print(f'ES_flag = {ES_flag}')
                if self.ES_flag == 0:
                    print("testing")
                    ES = EvolutionStrategy(3,15,fitness,[100,50,30,10,150], sigma = 15, frame = color_image)
                    
                    high_gradient, low_gradient, accumulator_threshold, self.minRadius, self.maxRadius = ES.optimize()
                    self.ES_flag = 1
                    #cycle to the next frame for processing        
                    frame = self.getNextFrame()
                    depth_image, color_image = self.extractImagesFromFrame(frame)
                elif self.ES_flag == 3:
                    print("here")
                    self.param = list(map(int, input("Type parameter numbers with space: ").split()))
                    high_gradient, low_gradient, accumulator_threshold, self.minRadius, self.maxRadius = self.param
                    print("Tune Again?")
                    self.ES_flag = input("If done enter 1. If not, enter 3")
                else:
    #                    param = [50,100,20,3,25]
                    high_gradient, low_gradient, accumulator_threshold, self.minRadius, self.maxRadius = self.param

            except:
                if self.ES_flag == 0:
                    print("test fail")
                else: 
                    print("look at lines 172 to 186")
            
            rounds = FindRounds(aruco_image[0], self.old_circles, high_gradient, low_gradient, accumulator_threshold, self.minRadius, self.maxRadius, self.coordinates, True)
            output = rounds.output()

            rounds_image = output[0]
            circles = output[1]
            self.old_circles = output[2]
            printables = output[3]
            reset_num = output[4]
            edge_img = output[5]
            circles = output[6]


            # Finding the Rotation Matrix for of the Camera wrt the world
            f_x= depth_intrinsics.fx
            f_y= depth_intrinsics.fy
            c_x= depth_intrinsics.ppx
            c_y = depth_intrinsics.ppy
            camera_matrix = np.array([[f_x,0,c_x],[0,f_y,c_y],[0 ,0 ,1 ]])
            dist_coeff= np.array([[depth_intrinsics.coeffs[0],depth_intrinsics.coeffs[1],depth_intrinsics.coeffs[2],depth_intrinsics.coeffs[3],depth_intrinsics.coeffs[4]]])
            
            if aruco_image[1] is not None:
                aruco = []
                for i in range(len(aruco_image[1])):
                    ret = cv2.aruco.estimatePoseSingleMarkers(aruco_image[1][i], 0.05, camera_matrix, dist_coeff)
                    rvec = ret[0][0, 0, :]
                    tvec = ret[1][0, 0, :]
                    topRight, bottomRight, bottomLeft, topLeft, cX, cY = aruco_image[3][i]
                    aruco.append((topRight, bottomRight, bottomLeft, topLeft, cX, cY, aruco_image[2][i], camera_matrix, dist_coeff, rvec, tvec))
            else:
                aruco = None
            # if aruco_image_2[1] is not None:
            #     ret = cv2.aruco.estimatePoseSingleMarkers(aruco_image_2[1], 0.05, camera_matrix, dist_coeff)
            #     rvec = ret[0][0, 0, :]
            #     tvec = ret[1][0, 0, :]
            #     topRight_2, bottomRight_2, bottomLeft_2, topLeft_2, cX_2, cY_2 = aruco_image_2[3]
            #     aruco_2 = topRight_2, bottomRight_2, bottomLeft_2, topLeft_2, cX_2, cY_2, aruco_image_2[2], camera_matrix, dist_coeff, rvec, tvec
            # else:
            #     aruco_2 = None
            self.coordinates.transform(depth_frame, depth_intrinsics)
            x = 324
            y = 285
            depth = depth_frame.get_distance(x, y)
            points = [[0.0, -0.05, 0.85],
                      [0.0, -0.1, 0.85],
                      [0.0, -0.15, 0.85],
                      [0.0, -0.2, 0.85],
                      [0.0, -0.25, 0.85],
                      [0.0, -0.3, 0.85],
                      [0.05, 0.0, 0.85],
                      [0.1, 0.0, 0.85],
                      [0.15, 0.0, 0.85],
                      [0.2, 0.0, 0.85],
                      [0.25, 0.0, 0.85],
                      [0.3, 0.0, 0.85],
                      [0.05, -0.05, 0.85],
                      [0.1, -0.05, 0.85],
                      [0.15, -0.05, 0.85],
                      [0.2, -0.05, 0.85],
                      [0.25, -0.05, 0.85],
                      [0.3, -0.05, 0.85],
                      [0.05, -0.1, 0.85],
                      [0.1, -0.1, 0.85],
                      [0.15, -0.1, 0.85],
                      [0.2, -0.1, 0.85],
                      [0.25, -0.1, 0.85],
                      [0.3, -0.1, 0.85],
                      [0.05, -0.15, 0.85],
                      [0.1, -0.15, 0.85],
                      [0.15, -0.15, 0.85],
                      [0.2, -0.15, 0.85],
                      [0.25, -0.15, 0.85],
                      [0.3, -0.15, 0.85],
                      [0.05, -0.2, 0.85],
                      [0.1, -0.2, 0.85],
                      [0.15, -0.2, 0.85],
                      [0.2, -0.2, 0.85],
                      [0.25, -0.2, 0.85],
                      [0.3, -0.2, 0.85],
                      [0.05, -0.25, 0.85],
                      [0.1, -0.25, 0.85],
                      [0.15, -0.25, 0.85],
                      [0.2, -0.25, 0.85],
                      [0.25, -0.25, 0.85],
                      [0.3, -0.25, 0.85],
                      [0.05, -0.3, 0.85],
                      [0.1, -0.3, 0.85],
                      [0.15, -0.3, 0.85],
                      [0.2, -0.3, 0.85],
                      [0.25, -0.3, 0.85],
                      [0.3, -0.3, 0.85]]
            newCoord = rs.rs2_project_point_to_pixel(color_intrinsics, [0.0, 0.0, 0.85])
            cv2.circle(color_image, (np.int16(newCoord[0]), np.int16(newCoord[1])), 1, (0,0,255), 1)
            scale = 0.97
            for point in points:
                newCoord = rs.rs2_project_point_to_pixel(color_intrinsics, point)
                new_point = [point[0] * scale, point[1] * scale, point[2]]
                scaled_coord = rs.rs2_project_point_to_pixel(color_intrinsics, new_point)
                cv2.circle(color_image, (np.int16(newCoord[0]), np.int16(newCoord[1])), 1, (0,0,255), 1)
                cv2.circle(color_image, (np.int16(scaled_coord[0]), np.int16(scaled_coord[1])), 1, (255,0,0), 1)
            
            
            robot = rounds_image
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

            self.tracker.updateTrajectory(frame, result)
            # self.tracker.updateTrajectory(frame, result_2)
            
            # if not self.event.is_set():
            self.vq.put([aruco, color_image, depth_colormap, circles])
            points = []
            for key, value in self.coordinates.items():
                points.append(f"{key}:{value}")
            # print(points)
            self.pq.put(points)
