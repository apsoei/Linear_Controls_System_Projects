# LQR optimal controller

# Import libraries
import numpy as np
from base_controller import BaseController
from lqr_solver import dlqr, lqr
from scipy.linalg import solve_continuous_lyapunov, solve_lyapunov, solve_discrete_lyapunov
from math import cos, sin
import numpy as np
from scipy import signal

class LQRController(BaseController):
    """ The LQR controller class.

    """

    def __init__(self, robot, lossOfThurst=0):
        """ LQR controller __init__ method.

        Initialize parameters here.

        Args:
            robot (webots controller object): Controller for the drone.
            lossOfThrust (float): percent lost of thrust.

        """
        super().__init__(robot, lossOfThurst)

        # define integral error
        self.int_e1 = 0
        self.int_e2 = 0
        self.int_e3 = 0
        self.int_e4 = 0

        # define K matrix
        self.K = None

    def initializeGainMatrix(self):
        """ Calculate the gain matrix.

        """

        # ---------------|LQR Controller|-------------------------
        # Use the results of linearization to create a state-space model

        n_p = 12 # number of states
        m = 4 # number of integral error terms

        # ----------------- Your Code Here ----------------- #
        # Compute the discretized A_d, B_d, C_d, D_d, for the computation of LQR gain 

        # ----------------- Your Code Ends Here ----------------- #
        delT = 1e-3*self.timestep

        # Initialize Ap,Bp,Cp,Dp
        A = np.zeros((16,16))
        B = np.zeros((16,4))
        C = np.zeros((4,16))
        D = np.zeros((4,8))
        for i in range(0,6):
            A[i][i+6]=1
        A[6][4]  = self.g
        A[7][3]  = -self.g
        B[8][0]  = 1/self.m
        B[9][1]  = 1/self.Ix
        B[10][2] = 1/self.Iy
        B[11][3] = 1/self.Iz
        

                
        Bc = np.zeros((12,4))
        Bc=np.vstack((Bc,-np.eye(4)))
        B = np.concatenate((B,Bc),axis=-1)
        
        for i in range(0,4):
            C[i][i] = 1

        # for i in range(12,16):
        for i in range(0,4):
            A[i+12][i] = 1
            
        CT = signal.StateSpace(A, B, C, D)
        DT = CT.to_discrete(delT)
        # DT = signal.StateSpace(A, B, C, D, dt=delT)        
        A_d = DT.A
        B_d = DT.B[:,:4]
        # B_d = DT.B
        # print(B_d)
        C_d = DT.C
        D_d = DT.D[:,:4]
        # print(C_d)
        # D_d = DT.D



        # -----------------    Example code     ----------------- #
        max_pos = 15.0
        max_ang = 0.2 * self.pi
        max_vel = 6.0
        max_rate = 0.015 * self.pi
        max_eyI = 3. 

        max_states = np.array([0.1 * max_pos, 0.1 * max_pos, max_pos,
                            max_ang, max_ang, max_ang,
                            0.5 * max_vel, 0.5 * max_vel, max_vel,
                            max_rate, max_rate, max_rate,
                            0.1 * max_eyI, 0.1 * max_eyI, 1 * max_eyI, 0.1 * max_eyI])

        max_inputs = np.array([0.2 * self.U1_max, self.U1_max, self.U1_max, self.U1_max])

        

        # -----------------  Example code Ends ----------------- #
        # ----------------- Your Code Here ----------------- #
        # Come up with reasonable values for Q and R (state and control weights)
        # The example code above is a good starting point, feel free to use them or write you own.
        # Tune them to get the better performance

        Q = np.diag(1/max_states**2)
        R = np.diag(1/max_inputs**2)*0.5
        
        # ----------------- Your Code Ends Here ----------------- #
        # Q = np.eye(16)*0.1
        # R = np.eye(4)*0.5
        # solve for LQR gains   
        [K, _, _] = dlqr(A_d, B_d, Q, R)

        self.K = -0.8*K

    def update(self, r):
        """ Get current states and calculate desired control input.

        Args:
            r (np.array): reference trajectory.

        Returns:
            np.array: states. information of the 16 states.
            np.array: U. desired control input.

        """

        # Fetch the states from the BaseController method
        x_t = super().getStates()

        # update integral term
        self.int_e1 += float((x_t[0]-r[0])*(self.timestep*1e-3))
        self.int_e2 += float((x_t[1]-r[1])*(self.timestep*1e-3))
        self.int_e3 += float((x_t[2]-r[2])*(self.timestep*1e-3))
        self.int_e4 += float((x_t[5]-r[3])*(self.timestep*1e-3))

        # Assemble error-based states into array
        error_state = np.array([self.int_e1, self.int_e2, self.int_e3, self.int_e4]).reshape((-1,1))
        states = np.concatenate((x_t, error_state))

        # calculate control input
        U = np.matmul(self.K, states)
        U[0] += self.g * self.m

        # Return all states and calculated control inputs U
        return states, U