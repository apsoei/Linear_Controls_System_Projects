import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.linalg import svd
import control


Ca = 20000
m = 1888.6
Lf = 1.55
Lr = 1.39
Iz = 25854
g = 9.81
Vx = np.asarray([2,5,8])


# P2: PROBLEM 1

for i in Vx:
    A = np.array([[0,            1,     0,                   0],
                [0, -4*Ca/(m*i),4*Ca/m,-2*Ca*(Lf-Lr)/(m*i)],
                [0,            0,     0,                   1],
                [0, -2*Ca*(Lf-Lr)/(Iz*i),2*Ca*(Lf-Lr)/Iz,-2*Ca*(Lf**2+Lr**2)/(Iz*i)]])

    B = np.array([[0,         0],
                [2*Ca/m,    0],
                [0,         0],
                [2*Ca*Lf/Iz,0]])
    AB = np.matmul(A,B)
    A2B = np.matmul(A,AB)
    A3B = np.matmul(A,A2B)

    C = np.eye(4)
    CA = np.matmul(C,A)
    CA2 = np.matmul(CA,A)
    CA3 = np.matmul(CA2,A)

    P = np.concatenate((B,AB),axis=1)
    P = np.concatenate((P,A2B),axis=1)
    P = np.concatenate((P,A3B),axis=1)
    print("Rank of P = ",np.linalg.matrix_rank(P))
    print(np.shape(P))
    Q = np.vstack((C,CA))
    Q = np.vstack((Q,CA2))
    Q = np.vstack((Q,CA3))
    print("Rank of Q = ",np.linalg.matrix_rank(Q))
    print(np.shape(Q))
print("Both controllable and observable for given x velocities\n")

    




# P2: PROBLEM 1
num = 50
Vx = np.linspace(1,40,num)
sigDivide = np.zeros((num,1))
pole1 = np.zeros((num,1))
pole2 = np.zeros((num,1))
pole3 = np.zeros((num,1))
pole4 = np.zeros((num,1))

# (a)
index = 0
for v in Vx:
    A = np.array([[0,            1,     0,                   0],
                [0, -4*Ca/(m*v),4*Ca/m,-2*Ca*(Lf-Lr)/(m*v)],
                [0,            0,     0,                   1],
                [0, -2*Ca*(Lf-Lr)/(Iz*v),2*Ca*(Lf-Lr)/Iz,-2*Ca*(Lf**2+Lr**2)/(Iz*v)]])
    B = np.array([[0,         0],
                [2*Ca/m,    0],
                [0,         0],
                [2*Ca*Lf/Iz,0]])
    C = np.eye(4)
    D = np.zeros((4,2))

    AB = np.matmul(A,B)
    A2B = np.matmul(A,AB)
    A3B = np.matmul(A,A2B)

    P = np.concatenate((B,AB),axis=1)
    P = np.concatenate((P,A2B),axis=1)
    P = np.concatenate((P,A3B),axis=1)

    U, sigma, V = np.linalg.svd(P)
    sigDivide[index] = sigma[0]/sigma[3]
    sys = control.StateSpace(A,B,C,D)
    pole1[index] = control.pole(sys)[0]
    pole2[index] = control.pole(sys)[1]
    pole3[index] = control.pole(sys)[2]
    pole4[index] = control.pole(sys)[3]
    index+=1
    # print(sigDivide)

plt.figure()
plt.title("Log10(Largest Sigma / Smallest Sigma) vs Vx")
plt.plot(Vx,sigDivide)
plt.xlabel("Vx [m/s]")
plt.ylabel("log10(sig1/sign)")
plt.grid()
plt.savefig("P2_Q1_a",bbox_inches='tight')
plt.show()    

# (b)
fig, axs = plt.subplots(2, 2,figsize=(10,10))
axs[0, 0].plot(Vx, pole1)
axs[0, 0].set_title('1st pole')
axs[0, 1].plot(Vx, pole2, 'tab:orange')
axs[0, 1].set_title('2nd pole')
axs[1, 0].plot(Vx, pole3, 'tab:green')
axs[1, 0].set_title('3rd pole')
axs[1, 1].plot(Vx, pole4, 'tab:red')
axs[1, 1].set_title('4th pole')
axs[0, 0].grid()
axs[0, 1].grid()
axs[1, 0].grid()
axs[1, 1].grid()
for ax in axs.flat:
    ax.set(xlabel='Vx [m/s]', ylabel='Re(poles)')
# Hide x labels and tick labels for top plots and y ticks for right plots.
# for ax in axs.flat:
#     ax.label_outer()
plt.savefig("P2_Q1_b",bbox_inches = 'tight')
plt.show()