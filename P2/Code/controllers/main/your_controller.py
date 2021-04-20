# Fill in the respective functions to implement the controller

# Import libraries
import numpy as np
from base_controller import BaseController
from scipy import signal, linalg
from util import *
from scipy.ndimage import gaussian_filter1d

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
        self.prev_e1 = 0
        self.prev_e1dot = 0
        self.prev_e2 = 0
        self.prev_e2dot = 0
        self.previousXdotError = 0

        # self.integralXdotError = 0
        
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

        # Design your controllers in the spaces below. 
        # Remember, your controllers will need to use the states
        # to calculate control inputs (F, delta). 

        A = np.array([[0,            1,     0,                   0],
                    [0, -4*Ca/(m*xdot),4*Ca/m,-2*Ca*(lf-lr)/(m*xdot)],
                    [0,            0,     0,                   1],
                    [0, -2*Ca*(lf-lr)/(Iz*xdot),2*Ca*(lf-lr)/Iz,-2*Ca*(lf**2+lr**2)/(Iz*xdot)]])
        # B = np.array([[0,         0],
                      # [2*Ca/m,    0],
                      # [0,         0],
                      # [2*Ca*lf/Iz,0]])
        B = np.array([[0       ],
                      [2*Ca/m,   ],
                      [0       ],
                      [2*Ca*lf/Iz]])
        C = np.eye(4)
        D = np.zeros((4,2))
        # AB = np.matmul(A,B)
        # A2B = np.matmul(A,AB)
        # A3B = np.matmul(A,A2B)


        # P = np.concatenate((B,AB),axis=1)
        # P = np.concatenate((P,A2B),axis=1)
        # P = np.concatenate((P,A3B),axis=1)
        _, node = closestNode(X, Y, trajectory)
        forwardIndex = 50
        try:
            psiDesired = np.arctan2(trajectory[node+forwardIndex,1]-Y, \
                                   trajectory[node+forwardIndex,0]-X)
        except:
            psiDesired = np.arctan2(trajectory[-1,1]-Y, \
                                  trajectory[-1,0]-X)
        
        
        
        poles = np.array([-7, -5, -1, -0.13])
        
        
        # sys = signal.StateSpace(A,B,C,D)
        # poles = sys.poles
        K = signal.place_poles(A, B, poles, method='YT')
        
        # print('Computed poles = ',K.computed_poles)
        K = 0.006*K.gain_matrix
      

        try:
            e1 = np.sqrt((trajectory[node+forwardIndex,1]-Y)**2 + \
                (trajectory[node+forwardIndex,0]-X)**2)

        except:
            e1 = np.sqrt((trajectory[-1,1]-Y)**2 + \
                (trajectory[-1,0]-X)**2)
                


        e1dot = (e1 - self.prev_e1)
        
        e2 = wrapToPi(psiDesired-psi)
        
        
        # e2 = psiDesired-psi
        # print("e2 = ",e2)
        
        e2dot = e2 - self.prev_e2/delT
        
        
        # print("e2dot = ",e2dot)
        self.prev_e1 = e1
        self.prev_e2 = e2
        # print('e2 = ',e2)
        # print('psi = ',psi)
        
        
        
        states = np.array([e1,e1dot,e2,e2dot])
        states = np.reshape(states,(4,1))
        # print(np.shape(states))
        # if sum(states)==0: U = 0
        U = -K@states
        # print('Delta = ',U)
        # print("e1,e1dot,e2,e2dot =",states)

        # ---------------|Lateral Controller|-------------------------
        
        # Please design your lateral controller below.
        
        # U = np.matmul(-K,)
        # if abs(e2) < 0.5 and abs(e2dot)<5: delta = np.pi/2
        delta = float(U)






        

        # ---------------|Longitudinal Controller|-------------------------
        
        # Please design your longitudinal controller below.
        kp = 230
        ki = 17
        kd = 120

        # Reference value for PID to tune to
        desiredVelocity = 8.5

        xdotError = (desiredVelocity - xdot)
        self.integralXdotError += xdotError
        derivativeXdotError = xdotError - self.previousXdotError
        self.previousXdotError = xdotError

        F = kp*xdotError + ki*self.integralXdotError*delT + kd*derivativeXdotError/delT
        
        # F = 5000
       

        # Return all states and calculated control inputs (F, delta)
        return X, Y, xdot, ydot, psi, psidot, F, delta
