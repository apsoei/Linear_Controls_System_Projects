import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import control

Ca = 20000
m = 1888.6
lf = 1.55
lr = 1.39
Iz = 25854
g = 9.81
xdot = 1
A = np.array([[0,            1,     0,                   0],
            [0, -4*Ca/(m*xdot),4*Ca/m,-2*Ca*(lf-lr)/(m*xdot)],
            [0,            0,     0,                   1],
            [0, -2*Ca*(lf-lr)/(Iz*xdot),2*Ca*(lf-lr)/Iz,-2*Ca*(lf**2+lr**2)/(Iz*xdot)]])
# B = np.array([[0,         0],
#                 [2*Ca/m,    0],
#                 [0,         0],
#                 [2*Ca*lf/Iz,0]])
B = np.array([[0         ],
                [2*Ca/m    ],
                [0         ],
                [2*Ca*lf/Iz]])
C = np.eye(4)
D = np.zeros((4,1))
# AB = np.matmul(A,B)
# A2B = np.matmul(A,AB)
# A3B = np.matmul(A,A2B)

# P = np.concatenate((B,AB),axis=1)
# P = np.concatenate((P,A2B),axis=1)
# P = np.concatenate((P,A3B),axis=1)

poles1 = np.array([-1, -2, -12, -3])
poles2 = np.array([-1, -2, -12, -3])/10
# poles = np.array([-1, -2, -4, -5])
K1 = signal.place_poles(A, B, poles1, method='YT')
K2 = signal.place_poles(A, B, poles2, method='YT')

print(K1.gain_matrix)
print(K2.gain_matrix)




# for i  in range(0,4):
#     print(K.computed_poles[i])

# K = control.place(A, B, poles)
# print(K.gain_matrix)
# print(np.shape(K))
# test_array = np.array([0,0,0,0])
# print(np.shape(test_array))
# print(-np.matmul(K,test_array))