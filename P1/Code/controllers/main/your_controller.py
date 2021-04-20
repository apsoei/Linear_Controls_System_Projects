# Fill in the respective functions to implement the controller

# Import libraries
import numpy as np
from base_controller import BaseController
from scipy import signal, linalg
from util import *

# CustomController class (inherits from BaseController)
class CustomController(BaseController):

    def __init__(self, trajectory):

        super().__init__(trajectory)

        # Define constants
        # These can be ignored in P1
        self.lr = 1.39
        self.lf = 1.55
        self.Ca = 20000
        self.Iz = 25854
        self.m = 1888.6
        self.g = 9.81

        # Add additional member variables according to your need here.
        
        self.de = 0
        self.e = 0
        self.sumE = 0
        self.kp = 1.85
        self.ki = 0.017
        self.kd = 3.925
        self.CTE = 0
        self.initialCTE = 5
        
        self.de2 = 0
        self.e2 = 0
        self.sumE2 = 0
        self.kp2 = 0.05
        self.ki2 = 0.0002
        self.kd2 = 0.5
        self.CTE2 = 0
        
    def cross_product(self, current, car, Ahead):
        X = Ahead - current
        Y = car - current
        return np.cross(X,Y)

    def update(self, timestep):

        trajectory = self.trajectory

        lr = self.lr
        lf = self.lf
        Ca = self.Ca
        Iz = self.Iz
        m = self.m
        g = self.g
        
        # Fetch the states from the BaseController method
        delT, X, Y, xdot, ydot, psi, psidot = super().getStates(timestep)
        
        CTE, minIndex = closestNode(X,Y,trajectory)
        
        
        
        
        
          
        # de = (self.CTE - CTE)/delT

        # self.e = self.CTE - CTE
        # sumE = self.e*CTE+self.sumE
        # self.CTE = CTE
        
       

        # print("CTE, minIndex =",CTE,minIndex)

                

        
        
        
        # Design your controllers in the spaces below. 
        # Remember, your controllers will need to use the states
        # to calculate control inputs (F, delta). 

        # ---------------|Lateral Controller|-------------------------
        
        # Please design your lateral controller below.
        
        # maxIndex = 8152
        
        point1 = trajectory[minIndex]
        point2 = trajectory[minIndex+25]


            
                 
        cross_val = self.cross_product(point1, np.array([X,Y]), point2)
       
        if cross_val > 0:
            pn = -1
        else:
            pn = 1
        
        # CTE = CTE*pn*abs(cross_val) - self.initialCTE
        CTE = CTE*pn - self.initialCTE

        self.de = (CTE - self.CTE)/delT
        self.e = CTE
        if(abs(self.e) < 0.3): self.sumE = 0
        self.sumE = self.e*delT+self.sumE
        self.CTE = CTE
        
        # DELTA
        delta = self.kp*self.e + self.ki*self.sumE + self.kd*self.de

        
        

        # ---------------|Longitudinal Controller|-------------------------
        
        # Please design your longitudinal controller below.
        
        self.de2 = (- abs(delta) - self.CTE2)/delT
        self.CTE2 = -abs(delta)
        
        self.e2 = self.CTE2
        self.sumE2= self.e2*delT+self.sumE
        
        # kp = 10, kd =5, ki = 2
        # error = closestNode(X,Y,trajectory)
        
        

        delta2 = self.kp2*self.e + self.ki2*self.sumE + self.kd2*self.de
        
        # FORCE
        F = 3000 + delta2
        
        # abs(psidot)>0.4
        if(xdot>12):
            F = -200
        if(xdot> 4 and abs(CTE)>0.35):
            F = -1500
        if(xdot< 2):
            F = 2000
        # -----------------------------------------------------
        # print("xdot = ",xdot)
        # print("psi = ",psi)
        # print(CTE)
        
        
        # -----------------------------------------------------
        # print("CTE = ",CTE)
        # print("delta = ",delta)
        # print("velocity = ",xdot*xdot+ydot*ydot)
        # print("Current Throttle = ", F)
        # print("delta2 =", delta2)
        # print("Steering vel =", psidot)
        # print("Point1, Point2 = ",point1, point2)
        # print("Difference in Angle = ", cross_val)
        
        # print("xdot = ",xdot)
        # if(self.sumE==0): print("RESETTING SUM_E VAL")
        # -----------------------------------------------------
        # print("DELTA Y = ",Del_Y)
        
        
        
        
        

        # Return all states and calculated control inputs (F, delta)
        return X, Y, xdot, ydot, psi, psidot, F, delta
