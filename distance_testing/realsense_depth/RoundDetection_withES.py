# -*- coding: utf-8 -*-
"""
Created on Fri Nov  5 00:05:56 2021

@author: isaac.hagberg
"""
#for the 155 round radius
# import os
# os.chdir("C:\\Users\\isaac.hagberg\\OneDrive - West Point\\Documents\\Firstie Year- 2\\CAPSTONE\\RealsenseArUcoTracking\\RealsenseArUcoTracking")


import cv2
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
from TuningHoughTranform_copyForIntegration import *

#adjust these for the distance function and votecounter
tol = 20 #[pixels]
votecount_max = 25 #needs 5 votes to be printed to the screen

reset_num = 0
reset_threshold = 50
################################################
# '''Here are the inputs that need tuning'''
# referenceOBJ = 6.02 / 2 # [cm]
# dp = 1  #Inverse ratio of the accumulator resolution to the image resolution
# high_gradient = 100 #higher threshold of the two passed to the Canny() edge detector
# low_gradient = high_gradient/2
# accumulator_threshold = 30
# minRadius = 10 #use the first plots to get a rough estimate of radius size
# maxRadius = 35
################################################
#input the name of the image
class FindRounds(object):

    def __init__(self, frame, old_circles, high_gradient, low_gradient, accumulator_threshold, minRad, maxRad):
        dp = 1  #Inverse ratio of the accumulator resolution to the image resolution
        self.high_gradient = high_gradient
        self.low_gradient = low_gradient
        self.accumulator_threshold = accumulator_threshold
        self.minRadius = minRad
        self.maxRadius = maxRad
        #circles2 is a global variable used to keep track of the persistent circles in each frame
        self.old_circles = old_circles
        self.img = frame
        # self.reset_num = 3
        # ES = EvolutionStrategy(3,15,fitness,[100,50,30,10,35], sigma = 15, frame)
        # high_gradient, low_gradient, accumulator_threshold, minRadius, maxRadius = ES.optimize()
        # self.original = cv2.imshow("Oringinal Frame", frame)
        self.copy = frame
        self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        self.blur = cv2.medianBlur(self.gray,5) #may need to play with the kernel size (think aperature) of the blur
        self.edge = cv2.Canny(self.blur, low_gradient, high_gradient) #this mimics the canny method used in circles where low_gradient = high/2
        self.circles = cv2.HoughCircles(self.blur, cv2.HOUGH_GRADIENT, 1, frame.shape[0]/64,param1 = self.high_gradient, param2 = self.accumulator_threshold, minRadius = round(self.minRadius), maxRadius = round(self.maxRadius))

        # self.edge = edge(self)   This are created in the methods below
        # self.referenceOBJ = refOBJ
        # self.circles = findCircles(self)
    ################################################
    # '''Here are the inputs that need tuning'''
    # referenceOBJ = 6.02 / 2 # [cm]
    # dp = 1  #Inverse ratio of the accumulator resolution to the image resolution
    # high_gradient = 100 #higher threshold of the two passed to the Canny() edge detector
    # low_gradient = high_gradient/2
    # accumulator_threshold = 30
    # minRadius = 10 #use the first plots to get a rough estimate of radius size
    # maxRadius = 35
    ################################################
    
    
    
    def originalView(self):
        plt.imshow(self.img)
        plt.show()
        return plt.imshow(self.img)
        
    def grayscaleView(self):
        plt.imshow(self.gray)
        plt.show()
        return plt.imshow(self.gray)
        
    def blurView(self):
        plt.imshow(self.blur)
        plt.show()
        return plt.imshow(self.blur)
    
    def edgeView(self):
        plt.imshow(self.edge)
        plt.show()
        return plt.imshow(self.edge)
    
    def midpoint(ptA, ptB):
        ptA = float(ptA)
        ptB = float(ptB)
        return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)
    
        
    def drawCircles(self):
        #draw the circles
        circles = self.circles
        if circles is not None:
            circles = np.uint16(np.around(circles))
            self.circles = self.circles[0]
            for i in circles[0,:]:
                #draw outer circle
                color1 = (0,255,0)
                cv2.circle(self.img, (i[0], i[1]), i[2], color1, 2) #last var is thickness
                
                #draw the center with a small circle
                color2 = (0,0,255)
                cv2.circle(self.img, (i[0], i[1]), 2, color2, 3)
    
    def estimatedDistances(self, num1, num2):
        xy = self.circles[0,:,:]
        #converet from uint8to int so that we can work with negative numbers
        xy = np.array(xy, dtype = int)
        i = num1
        j = num2
        referenceRatio = reference(xy, self.refOBJ)
        D = distance(xy,i,j) * referenceRatio
        (mX, mY) = midpoint((xy[i,0], xy[i,1]),(xy[j,0], xy[j,1]))
        cv2.putText(self.img, "{:.2f}in".format(D), (int(mX+10), int(mY+5)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2)
        cv2.line(self.img, (xy[i,0], xy[i,1]),(xy[j,0], xy[j,1]), (255, 255, 0), 2)
        
        
    def reference(xy,measured):
        '''Calculate the pixel to meter ratio'''
        i = 1 #use the first detected circle 
        radius = np.array((xy[i,2]))  # last attempt: (xy[i,0] + xy[i,2], xy[i,1]+ xy[i,2])
        
        referenceOBJ = measured / radius
        return referenceOBJ
        
        
        
    def viewAll(self):
        fig,ax = plt.subplots(nrows = 1,ncols = 3,figsize=(12,5))
        ax[0].imshow(self.original)
        ax[0].title.set_text("Original Image")
        ax[0].set_xticks([])
        ax[0].set_yticks([])
        ax[1].imshow(self.gray)
        ax[1].title.set_text("Grayscale Image")
        ax[1].set_xticks([])
        ax[1].set_yticks([])
        
        ax[2].imshow(self.edge)
        ax[2].title.set_text("Edges")
        ax[2].set_xticks([])
        ax[2].set_yticks([])
        
    def results(self):
        
        self.drawCircles()
        # self.estimatedDistances(0,1) #insert the object you want to measure the distance between
        winname = 'Detected Circles'
        cv2.namedWindow(winname, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(winname, 600,600)
        cv2.imshow(winname, self.img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    def distance(self, x,y, x0, y0):
        ret = sqrt((x-x0)**2 + (y-y0)**2)
        return ret

    def persistence(self, newcircle_data,oldcircle_data, votecount_max, printables):
        #calculate the average for x, y, and r
        # print(f'Before AVG = {oldcircle_data}')
        n = oldcircle_data[3]
        for z in range(3):
            oldcircle_data[z] = oldcircle_data[z]*n/(n+1) + newcircle_data[z]/(n+1)
        
        # print(f'After AVG: {oldcircle_data}')
        
        #increase the vote count
        oldcircle_data[3] += 1
        #check if the accumulator is high enough for printing
        # print(f'oldcirc data vote count = {oldcircle_data[3]}')
#        print(f'Circle Data: \n{self.old_circles}')
        if oldcircle_data[3] > votecount_max:
            printables.append(oldcircle_data)
        return oldcircle_data
    
    def matchingCircles(self):
        '''
        This function return the circles with a high enough vote count to be printed
        '''
        
        printables = []
        # print("Just before")
        print("=========================================")
        
        try:
            temp = np.array([])
            if len(self.old_circles) > 0:
                for idx, circ in enumerate(self.old_circles):
                    #add on the vote accumulator to the end of each array for detected circles
                    if len(circ) ==3:
                        add = np.append(circ, [1])
                        temp = np.append(temp,add)
                        
                    elif len(circ) ==4:
                        temp = np.append(temp, circ)
                temp = np.reshape(temp, [int(len(temp)/4),4])
                # print(f'Temp: {temp}')
                self.old_circles = temp
        except:
            pass
        
        try:
            if len(self.old_circles) == 0:
                self.old_circles = self.circles
                # print(f'gate 1: {self.old_circles}')
            elif len(self.old_circles) > 0 and (np.any(self.circles != None)) :
                for z, i in enumerate(self.circles):
                    # print(f'item in newcircle: {z}, item {i}')
                    x0,y0 = i[0], i[1]
                    flag_exist = 0
                    for q, j in enumerate(self.old_circles):
                                                          
                        # print(f'Oldcirc_list: {self.old_circles}')    
                            
                            
                            
                        # print(f'item in OLDcircle: {q}, item {j}')
                        x,y = j[0], j[1]
                        # print(f'idx: {q}\n\n\n')
                        # print(x,y,x0,y0)
                        # print(f'Distance: {self.distance(x,y, x0,y0)}')
                        if self.distance(x,y, x0,y0) < tol:
                            #persistence function. add to the global list
                            check = self.persistence(self.circles[z], self.old_circles[q], votecount_max, printables)
                            # print(f'persist: {check}')
                            flag_exist = 1
#                            break #this sbreak command will take care of duplicate, overlapping circles so that things aren't added twice
                                                    
                        else:
                            continue
                        
                        
                    #for loop complete.  if the circle does not match ANY of the other circles than append it to self.circles

                    if flag_exist == 0: 
                        #add the new circle into the group to be analyzed in future frames
                        #this may slow the program down significantly if the hyperparameters aren't tuned correctly
#                        print(f'\n\nThis is what you tried to append to old_circles:   {i}')
#                       input()

                        adder_circles = self.old_circles
#                        print(f'adder circles = {adder_circles}\n\n type of adder = {type(adder_circles)}')
                        i = np.append(i,1) #add on the accumulator
                        i = i.tolist()
#                        print(f'adding to it is = {i} with type = {type(i)}')
                        adder_circles = adder_circles.tolist()
#                        print(f'adding to it is = {adder_circles} with type = {type(adder_circles)}')
                        adder_circles.append(i)
#                        print(f'Final Form= {adder_circles} with type = {type(adder_circles)}')
                        
#                        input()
#                        adder_circles = np.append(adder_circles, i)
#                        print(adder_circles)
#                        print(f'self old circles before: {adder_circles}')
#                        self.old_circles = self.old_circles.append(i)
#                        self.old_circles = np.array(self.old_circles)
                        self.old_circles = adder_circles
#                        print(f'self oldcircles after: {self.old_circles}')
#                        
#                        input()
                print(f'Circle Data: \n{self.old_circles}')
                             
        except: 
            if self.old_circles is None:
                self.old_circles = self.circles
                # print(f'gate 2: {self.old_circles}')
                
                        
        return printables
            
        
    
    def reset(self):
        #reset the persistence data every 15 frames. Camera is set by the program at 30 Hz
        # print("HIPPOPOTOMUS")
        global reset_num
        global reset_threshold
        # reset_num += 1
        # print(f"reset num = {reset_num}")
        if reset_num >= reset_threshold: 
            
            reset_num = 0
            print("RESET\n")
            return True
        else:
            reset_num += 1
            print(f'reset_num = {reset_num}\n')
            return False
    
    
    def output(self):
        # self.reset()
        self.drawCircles()
        # print("checkmate")
        print(f'OldCirc: {self.old_circles}')
        print(f'NewCirc: {self.circles}')
        printables = self.matchingCircles()
        # print("winner")
        if self.reset():
            print("***************ENDEX \n")  
            printables= []
            if self.old_circles is not None:
                for oldcircle_data in self.old_circles:
                    if oldcircle_data[3] > votecount_max:
                        printables.append(oldcircle_data)
                printables = np.array(printables)
#            print(f'Printables = {printables}')
            #if printables = None then it will throw up an error so use try-except
        
            #sort printables by the x then the y data
            #order is reversed inside of the tuple. So insert the first column you want to sort by LAST.
            if len(printables)>1:                
                printables = printables[np.lexsort((printables[:,1],printables[:,0]))]
            #sort by x data
            # printables = printables[np.argsort(printables[:,0])]
            # #sort by y data
            # printables = printables[np.argsort(printables[:,0])]

            print(f'THIS IS IT!!!!\n {printables}')
#            input()
            #reset the circle persistence matrix
            self.old_circles = []
            
            return self.img, self.circles, self.old_circles, printables , reset_threshold, self.edge
        else:
            print("***************Carry On\n")
            return self.img, self.circles, self.old_circles, None , reset_num, self.edge
        
    #view what just happened
    
# color_image, circles, old_circles, printables = FindRounds(color_image, old_circles).output()

#Main

# #%%
# ################################################
# '''Here are the inputs that need tuning'''
# referenceOBJ = 6.02 / 2 # [cm]
# dp = 1  #Inverse ratio of the accumulator resolution to the image resolution
# high_gradient = 100 #higher threshold of the two passed to the Canny() edge detector
# low_gradient = high_gradient/2
# accumulator_threshold = 30
# minRadius = 10 #use the first plots to get a rough estimate of radius size
# maxRadius = 35
# ################################################
    
# refOBJ = 6.09/2 #[in]
# imageName = "DemoLive.jpg" 
# x = ImageProcessing(imageName, refOBJ)
# x.edgeView()
# x.originalView()
# x.grayscaleView()
# x.blurView()
# x.edgeView()
# x.drawCircles()
# x.viewAll()
# x.results()
# print(x.circles)
# circles = x.circles
# x.estimatedDistances(1,2)

# ###
# old_circles = []
# color_image = color_image
# x = FindRounds(color_image, old_circles) #.output()
# x.old_circles
# x.circles
# im, circ, old_c, printab = x.output()
# old_c

###

#x.results()
# aa = x.circles
# cv2.imshow("chuck", x.img)
# cv2.waitKey(0)



# x.originalView()
# x.results()
# frame = x.img
# x.viewAll()


# img = frame
# original = cv2.imshow("Oringinal Frame", img)
# copy = frame
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# blur = cv2.medianBlur(gray,5) #may need to play with the kernel size (think aperature) of the blur
# edge = cv2.Canny(blur, low_gradient, high_gradient) #this mimics the canny method used in circles where low_gradient = high/2
# circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1, frame.shape[0]/64, param1 = high_gradient, param2 = accumulator_threshold, minRadius = minRadius, maxRadius = maxRadius)






