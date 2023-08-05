# kf/second_order_kf.py
import numpy as np
import scipy.linalg as la
import control.matlab as ct
import matplotlib.pyplot as plt

# Define the system matrices and parameters
A = np.array([[0, 1], [0, 0.7114]])
Bw = np.array([[0.0384], [0.0722]])
C = np.array([[1, 0]])
D = 0
W = np.array([[1]])  # input noise variance
r = np.array([1, 5])

# Solving for Xss
Xss = la.solve_discrete_lyapunov(A, Bw @ Bw.T)  # XSS=A*Xss*A'+Bw*Bw'
X11 = Xss[0, 0]
V1 = r[0]**2 * X11
V2 = r[1]**2 * X11

# Steady-state KF gains and covariances
M1, _, F1 = ct.dare(A.T, C.T, Bw @ W @ Bw.T, V1)
F1 = F1.T  # Transpose to match matlab output
M2, _, F2 = ct.dare(A.T, C.T, Bw @ W @ Bw.T, V2)
F2 = F2.T  # Transpose to match matlab output
Z1 = M1 - M1 @ C.T @ la.inv(C @ M1 @ C.T + V1) @ C @ M1
Z2 = M2 - M2 @ C.T @ la.inv(C @ M2 @ C.T + V2) @ C @ M2

# Finite settling time observer not based on least square optimal stochastic estimation
F3 = np.array([1, 0.7114])
N = 200
w = np.random.randn(N)
v1 = np.random.randn(N) * (r[0] * np.sqrt(X11))
v2 = np.random.randn(N) * (r[1] * np.sqrt(X11))
Ts = 1
t = np.arange(200)

# Actual system response
sys = ct.ss(A, Bw, C, D, Ts)
y0, t0, x = ct.lsim(sys, w, t)  # assume E[x]=0
y1 = y0 + v1
y2 = y0 + v2

# Kalman Filter simulation
# Akf1 = np.array([[0, 1 - F1[0]], [0, 0.7114 - F1[1]]])
Akf1 = np.array([[0, 1 - F1[0, 0]], [0, 0.7114 - F1[1, 0]]])
Bkf1 = F1.reshape(-1, 1)
sys_kf1 = ct.ss(Akf1, Bkf1, C, D, Ts)
y_kf1, t1, x_kf1 = ct.lsim(sys_kf1, y1.reshape(-1), t)

Akf2 = np.array([[0, 1 - F2[0, 0]], [0, 0.7114 - F2[1, 0]]])
Bkf2 = F2.reshape(-1, 1)
sys_kf2 = ct.ss(Akf2, Bkf2, C, D, Ts)
y_kf2, t2, x_kf2 = ct.lsim(sys_kf2, y2.reshape(-1), t)

# Finite settling time observer simulation
Afsto = np.array([[0, 1 - F3[0]], [0, 0.7114 - F3[1]]])
Bfsto = F3.reshape(-1, 1)
sys_fsto = ct.ss(Afsto, Bfsto, C, D, Ts)
y_fsto1, t3, x_fsto1 = ct.lsim(sys_fsto, y1.reshape(-1), t)
y_fsto2, t4, x_fsto2 = ct.lsim(sys_fsto, y2.reshape(-1), t)

# Plot the results
plt.figure()
m = 8
plt.subplot(m, 1, 1)
plt.plot(t0, x[:, 0])
plt.title('$x_1$')
plt.subplot(m, 1, 2)
plt.plot(t0, v1)
plt.title('v')
plt.subplot(m, 1, 3)
plt.plot(t0, y1)
plt.title('y')
plt.subplot(m, 1, 4)
plt.plot(t1, x_kf1[:, 0])
plt.title('$\widehat x_1(k|k)$')
plt.subplot(m, 1, 5)
plt.plot(t0, x[:, 1])
plt.title('$x_2$')
plt.subplot(m, 1, 6)
plt.plot(t1, x_kf1[:, 1])
plt.title('$\widehat x_2(k|k)$')
plt.subplot(m, 1, 7)
plt.plot(t1, x_fsto1[:, 0])
plt.title('$\widehat x_1(k)$ FSTO')
plt.subplot(m, 1, 8)
plt.plot(t1, x_fsto1[:, 1])
plt.title('$\widehat x_2(k)$ FSTO')
plt.xlabel('time step: k')

plt.figure()
plt.subplot(m, 1, 1)
plt.plot(t0, x[:, 0])
plt.title('$x_1$')
plt.subplot(m, 1, 2)
plt.plot(t0, v2)
plt.title('v')
plt.subplot(m, 1, 3)
plt.plot(t0, y2)
plt.title('y')
plt.subplot(m, 1, 4)
plt.plot(t2, x_kf2[:, 0])
plt.title('$\widehat x_1(k|k)$')
plt.subplot(m, 1, 5)
plt.plot(t0, x[:, 1])
plt.title('$x_2$')
plt.subplot(m, 1, 6)
plt.plot(t2, x_kf2[:, 1])
plt.title('$\widehat x_2(k|k)$')
plt.subplot(m, 1, 7)
plt.plot(t4, x_fsto2[:, 0])
plt.title('$\widehat x_1(k)$ FSTO')
plt.subplot(m, 1, 8)
plt.plot(t4, x_fsto2[:, 1])
plt.title('$\widehat x_2(k)$ FSTO')
plt.xlabel('time step: k')

# Comparison of the direct output measurement with the KF
fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(t0, x[:, 0]-y1, 'r--', t1[:-1], x[:-1, 0]-x_kf1[1:, 0], 'k')
ax.legend(['$x_1-y$', '$x_1-\widehat x_1(k|k)$'])
ax.set_title(f'direct measurement error vs estimation error r={r[0]}')
ax.set_xlabel('k')
ax.set_ylabel('$x_1$')

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(t0, x[:, 0]-y2, 'r--', t2[:-1], x[:-1, 0]-x_kf2[1:, 0], 'k')
ax.legend(['$x_1-y$', '$x_1-\widehat x_1(k|k)$'])
ax.set_title(f'direct measurement error vs estimation error r={r[1]}')
ax.set_xlabel('k')
ax.set_ylabel('$x_2$')

# Comparison of the finite settling time observer with the KF
fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(t0[:-1], x[:-1, 1]-x_fsto1[1:, 1], 'r--',
        t2[:-1], x[:-1, 1]-x_kf1[1:, 1], 'k')
ax.legend(['FSTO', 'KF'])
ax.set_title(f'estimation error of the second state; r={r[0]}')
ax.set_xlabel('k')
ax.set_ylabel('$x_2$')

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(t0[:-1], x[:-1, 1]-x_fsto2[1:, 1], 'r--',
        t2[:-1], x[:-1, 1]-x_kf2[1:, 1], 'k')
ax.legend(['FSTO', 'KF'])
ax.set_title(f'estimation error of the second state; r={r[1]}')
ax.set_xlabel('k')
ax.set_ylabel('$x_2$')
