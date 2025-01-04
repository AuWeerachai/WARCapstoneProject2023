# -*- coding: utf-8 -*-
"""
Created on Mon Dec 27 13:47:34 2021

@author: isaac.hagberg
"""
import cv2
import numpy as np
import matplotlib.pyplot as plt
# from RoundDetection_withES import FindRounds


#define the objective function
#the obejctive will be an output of n-circles from the given image


#actual_num_circles = int(input("Input the number of circles in the image: "))



# image = cv2.imread("evol_strat.jpg")
# image = cv2.imread("DemoLive.jpg")
# image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
# ################################################
# '''Here are the inputs that need tuning'''
# dp = 1  #Inverse ratio of the accumulator resolution to the image resolution
# high_gradient = 100 #higher threshold of the two passed to the Canny() edge detector
# low_gradient = high_gradient/2
# accumulator_threshold = 50
# minRadius = 10 #use the first plots to get a rough estimate of radius size
# maxRadius = 100
################################################
#input the name of the image


#%%
def fitness(high_gradient, low_gradient, accumulator_threshold, minRadius, maxRadius, frame):
      
    # high_gradient = high_gradient
    # low_gradient = low_gradient
    # accumulator_threshold = accumulator_threshold
    # minRadius = minRad
    # maxRadius = maxRad
    frame_copy = frame.copy()
    try:
        obj = FindRounds(frame_copy, [], high_gradient, low_gradient,accumulator_threshold, minRadius, maxRadius)
        if obj.circles is not None:
            print("HIppo")
            ans = len(obj.circles[0]) - actual_num_circles
            obj.drawCircles()
            obj.originalView()
            return ans
            # obj.drawCircles()
            # obj.originalView()
            # return len(obj.circles[0]) - actual_num_circles 
            
        else:
            print("pottomos")
            # obj.originalView()
            #the return needs to be worse than the abs(actual) or else sigma decreases
            return actual_num_circles * 50
    except:
        # this is to catch any instances where the wrong parameters become negative. 
        # The Hough Tranform cannot handle negative parameter values for accum, minRad, or maxRad
        return 10000

# num= fitness(100,50,50,10,100)

# a.circles
# fitness(63.16999016 , -8.13963942 , 67.81169106 , 16.25282134, 146.30223233) #fitness 2
#%%

class EvolutionStrategy(object):
    def __init__(self,mu,lam,fitness,x0,sigma, frame):
        try:
            self.dimension = len(x0)
        except:
            self.dimension = 1
        self.frame = frame
        self.fitness = fitness
        self.curr_loc = np.array(x0)
        self.curr_fitness = self.fitness(self.curr_loc[0],self.curr_loc[1],self.curr_loc[2],self.curr_loc[3],self.curr_loc[4], self.frame)
        self.sigma = sigma
        self.mu = mu
        self.lam = lam
        self.offspring = []
        
    
    def __str__(self):
        res = ""
        res+=str(self.mu)+"/"+str(self.mu)+","+str(self.lam)+" ES\n"
        res+="Current Location: "+str(self.curr_loc)+"\n"
        res+="Current Fitness: "+str(self.curr_fitness)+"\n"
        res+="Current Sigma: "+str(self.sigma)
        return res
    
    def create_offspring(self):
        mutation = self.sigma*np.random.normal(0,1,(self.lam,self.dimension))
        
        
        # TODO: verify this code works 
        # normalize the data
        # high_gradient, low_gradient, accumulator_threshold, minRadius, maxRadius
        
        normalize = np.array([500,500,1000,1000,1000])
        
        # mutation = [a * b for a, b in zip(mutation,normalize)]
        # print(f'mutation = {mutation}')
        # mutation = np.array(mutation) 
        mutation = np.multiply(mutation,normalize)
        self.offspring = self.curr_loc+mutation
    
    def evaluate_offspring(self):
        self.offspring_fitness = np.zeros(self.lam)
        for i in range(len(self.offspring)):
            self.offspring_fitness[i] = self.fitness(self.offspring[i][0],self.offspring[i][1],self.offspring[i][2],self.offspring[i][3],self.offspring[i][4], self.frame)
            
    def selection(self):
        mu = self.mu
        idx = np.argsort(self.offspring_fitness)
        # print(f'idx: {idx} \n mu: {mu}')   
        # print(f'offspring: {self.offspring}')
        self.parents = self.offspring[idx[:mu]]  #,:]
    
    def adaption(self):
        Ps = (self.offspring_fitness<self.curr_fitness).sum()/self.lam #probability of success --> # of better offspring / total offspring
        self.sigma = self.sigma*0.9 if Ps < 1/5 else self.sigma / 0.9
        
    def recombination(self):
        self.curr_loc = self.parents.mean(axis = 0)
        self.curr_fitness = self.fitness(self.curr_loc[0],self.curr_loc[1],self.curr_loc[2],self.curr_loc[3],self.curr_loc[4], self.frame)
        
    def optimize(self, tol = 10E-4):
        tic = 0
        while self.sigma >tol and self.curr_fitness != 0 and tic < 20:
            tic +=1
            self.create_offspring()
            self.evaluate_offspring()
            self.selection()
            self.adaption()
            self.recombination()
            print(f'Loc: {self.curr_loc}\n fit: {self.curr_fitness}\n sigma: {self.sigma}')
            # input()
        print("we made it!")
        return self.curr_loc
    
    
    
    
    
    
    
    
    
    
    
    

#%%    


# ES = EvolutionStrategy(3,15,fitness,[50,40,20,10,60], sigma = 15, frame = color_image)
# high_gradient, low_gradient, accumulator_threshold, minRadius, maxRadius = ES.optimize()


### Establish the class instance


# np.random.seed(1000)
# #EvolutionStrategy(# of best children to pick, # of children, fit func, starting guess for parameters, step size)
# ES = EvolutionStrategy(3,10,fitness,[100,10,10,10,50], sigma = 10)  
# ES.curr_fitness
# print(ES)

#%%
# np.random.seed(1000)
# ES = EvolutionStrategy(3,15,fitness,[20,10,2,10,150], sigma = 15)
# high_gradient, low_gradient, accumulator_threshold, minRadius, maxRadius = ES.optimize()

#%%

# obj = FindRounds(image, [],63.16999016 , -8.13963942 , 67.81169106 , 16.25282134 ,0146.30223233)
# if obj.circles is not None:
#         obj.drawCircles()
#         obj.originalView()

