import numpy as np
from scipy.linalg import solve_continuous_lyapunov, solve_lyapunov, solve_discrete_lyapunov
from math import cos, sin
import numpy as np
import scipy.linalg

m = 0.4
d1x = 0.1122
d1y = 0.1515
d2x = 0.11709
d2y = 0.128
Ix = 0.000913855
Iy = 0.00236242
Iz = 0.00279965

# define constants
g = 9.81
ct = 0.00026
ctau = 5.2e-06
U1_max = 10
pi = 3.1415926535



A = np.zeros((16,16))
B = np.zeros((16,4))
C = np.zeros((4,16))
D = np.zeros((4,4))


for i in range(0,6):
    A[i][i+6]=1
A[6][4]  = g
A[7][3]  = -g
B[8][0]  = 1/m
B[9][1]  = 1/Ix
B[10][2] = 1/Iy
B[11][3] = 1/Iz
for i in range(0,4):
    C[i][i] = 1

for i in range(0,4):
    A[i+12][i] = 1

Q = np.eye(16)
R = np.eye(4)*100
print("--------")
print("--------")
print(np.shape(R))
print("--------")
print("--------")

# X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
X = scipy.linalg.solve_continuous_are(A, B, Q, R)
print(X)
print("--------")
print("--------")
print(np.shape(X))

#compute the LQR gain
K = np.matrix(scipy.linalg.inv(R)@(B.T@X))

eigVals, eigVecs = scipy.linalg.eig(A-B*K)

# Xd = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))

# #compute the LQR gain
# Kd = np.matrix(scipy.linalg.inv(B.T*X*B+R)*(B.T*X*A))

# eigVals, eigVecs = scipy.linalg.eig(A-B*K)