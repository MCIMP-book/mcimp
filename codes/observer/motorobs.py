# observer/motorobs.py
import numpy as np
import matplotlib.pyplot as plt
import control.matlab as ct

# Continuous-time system model
L = 1e-3; R = 1; J = 5e-5; B = 1e-4; K = 0.1

A = np.array([[-R/L, 0, -K/L], [0, 0, 1], [K/J, 0, -B/J]])
B = np.array([1/L, 0, 0]).reshape((3, 1))
C = np.array([0, 1, 0]).reshape((1, 3))
D = np.array([0])

# check original eigenvalues
print(np.linalg.eig(A))

# Observer design
# check observability
O = np.linalg.matrix_rank(np.concatenate(
    (C, C@A, C@(A@A)), axis=0))
print(O)
# or you can use np.block
O = np.linalg.matrix_rank(np.block([ [C], [C@A], [C@(A@A)] ]))
print(O)

pole_des = np.array([-500+250j, -500-250j, -1000])

# design observer by placing poles of A-LC
L = ct.place(A.T, C.T, pole_des).T
est_poles = np.linalg.eig(A - L@C)

# Simulation
Aaug = np.block([A, np.zeros((3, 3))])
Aaug = np.block([ [Aaug], [np.block([L@C, A - L@C])] ])
Baug = np.block([[B], [B]])
Caug = np.block([C, np.zeros((1, 3))])
Daug = np.array([0])
# Aaug = np.concatenate((A, np.zeros((3, 3))), axis=1)
# Aaug = np.concatenate((Aaug, np.concatenate(
#     (L@C, A - L@C), axis=1)), axis=0)
# Baug = np.concatenate((B, B), axis=0)
# Caug = np.concatenate((C, np.zeros((1, 3))), axis=1)
# Daug = np.array([0])


sys = ct.ss(Aaug, Baug, Caug, Daug)

x0 = np.array([10, 2, 10]); xhat0 = np.array([0, 0, 0]); X0 = np.array([x0, xhat0]).reshape((6, 1))

Tend = 0.03; amplitude = 10; initpha = 0; freq = 600
t = np.arange(0, Tend, 1e-4)

u = amplitude * np.sin(freq * t + initpha)

[Y, T, X] = ct.lsim(sys, u, t, X0)

plt.figure()
plt.subplot(3, 1, 1)
plt.plot(t, X[:, 0], t, X[:, 3], '--', linewidth=1.5)
plt.xlabel('time (sec)')
plt.legend(['$x_1 = i_a$', '$\hat x_1$'], fontsize=16)
plt.grid()
plt.ylabel('$x_1$', fontsize=16)
plt.title('States and their estimates')
plt.subplot(3, 1, 2)
plt.plot(t, X[:, 1], t, X[:, 4], '--', linewidth=1.5)
plt.xlabel('time (sec)')
plt.legend(['$x_2 = \\theta$', '$\hat x_2$'], fontsize=16)
plt.grid()
plt.ylabel('$x_2$', fontsize=16)
plt.subplot(3, 1, 3)
plt.plot(t, X[:, 2], t, X[:, 5], '--', linewidth=1.5)
plt.xlabel('time (sec)')
plt.legend(['$x_3 = \dot{\\theta}$', '$\hat x_3$'], fontsize=16)
plt.grid()
plt.ylabel('$x_3$', fontsize=16)
plt.show()
