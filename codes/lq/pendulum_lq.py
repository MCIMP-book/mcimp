# lq/pendulum_lq.py
# Python implementation of LQ in controlling a pendulum on a cart
import numpy as np
import scipy.integrate as integrate
import control.matlab as co
import matplotlib.pyplot as plt

I = 1       # Moment of Inertia
m = 1       # Mass of the Pendulum
M = 5       # Mass of the Cart
L = 2       # Length of the Pendulum
g = -9.8    # Acc. Due to gravity
d = 1       # Damping


def statespace(y, t):
    """ Creates State vector from given initial conditions.
        State Linear vector follows the equation Xdot = AX. This function creates the vector using
        the dynamic model of the system. This vector is passed through scipy.integrate.odeint functin
        which will integrate the vector and output states similar to ode45 in MATLAB.

        inputs:
            y = state vector that will be calculated by odeint
            t = time vector

        returns:
            dydt = State Vector

    """

    # Assigning a variable for sin theta for simplicity in writing equations
    Sy = np.sin(y[2])
    # Assigning a variable for sin theta for simplicity in writing equations
    Cy = np.cos(y[2])
    # D = m*L*L*(M+m*(1-Cy**2))   # Denominator of the equations of motion
    D = (M+m)*(I+m*L**2) - m**2*L**2*Cy**2
    dydt = np.zeros_like(y)     # Creating dydt vector same size/shape as y

    """ Arrays and matrices are treated differently in numpy and therefore need to be treated differently.
    We need to calculate the input u to the states and therefore need to convert the vectors to the same data type
    before proceeding"""

    y_mat = np.block([[y[0]], [y[1]], [y[2]], [y[3]]])

    inp = -Kf*(y_mat-yd_mat)
    # We cannot directly use inp to multiply in the equations and need to extract it from the 1x1 inp matrix
    u = inp[0, 0]

    # State Equations
    dydt[0] = y[1]
    dydt[1] = (1/D)*(-m**2*L**2*g*Cy*Sy + m*L**2 *
                     (m*L*y[3]**2*Sy - d*y[1])) + m*L*L*(1/D)*u
    dydt[2] = y[3]
    dydt[3] = (1/D)*((m+M)*m*g*L*Sy - m*L*Cy *
                     (m*L*y[3]**2*Sy - d*y[1])) - m*L*Cy*(1/D)*u

    return dydt


# Defining linearized state space model below and calculating state feedback control gains for optimal control

A = np.array([[0, 1, 0, 0], [0, -d/M, -m*g/M, 0],
             [0, 0, 0, 1], [0, -d/(M+L), -(m+M)*g/(M+L), 0]])
B = np.array([[0], [1/M], [0], [1/(M*L)]])
C = np.array([[1, 0, 0, 0],[0, 0, 1, 0]])
D = np.array([[0], [0]])

# Q and R for LQR
Q = np.block([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 10, 0], [0, 0, 0, 100]])
R = 0.0001

# Calculating state-feedback gain matrix
[K, S, E] = co.lqr(A, B, Q, R)
Kf = np.array([K[0, 0], K[0, 1], K[0, 2], K[0, 3]])

# Initializing the first base run

# Initial Conditions
y0 = np.array([0, 0, np.pi+0.2, 0])
yd = np.array([0, 0, np.pi, 0])     # Desired Postions
yd_mat = np.block([[yd[0]], [yd[1]], [yd[2]], [yd[3]]]
                   )    # making it a 4x1 Matrix

# Creating time vector
T_final = 15
t = np.arange(0, T_final, 0.01)

# Integrating and getting state vector
y = integrate.odeint(statespace, y0, t)

# Creating the Figure
fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                     position=[.15, .15, .75, .75])
ax.grid()

# Setting the axis to the max displacement x so the whole cart is visible
ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])

# Drawing the parts of the system

# Cart and Wheels. the code blow draws the initial postions of all the parts
cart_x = y[0, 0]
cart, = ax.plot([cart_x-3, cart_x+3], [0.5, 0.5], linewidth=3)
wheel1, = ax.plot(cart_x-2, [0.25], 'ok', ms=10)
wheel2, = ax.plot(cart_x+2, [0.25], 'ok', ms=10)

# Pendulum and rod
pend_x = cart_x + L*np.sin(y[0, 2])
pend_y = 1 - L*np.cos(y[0, 2])
pend_rod, = ax.plot([cart_x, pend_x], [0.5, pend_y], linewidth=3)
pend, = ax.plot(pend_x, pend_y, 'ro', ms=10)

# Plot time series

fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(411)
ax.grid()
ax.plot(t, y[:, 0], 'b', label='$x$')
ax.set_ylabel('$x$ (m)')
ax = fig.add_subplot(412)
ax.grid()
ax.plot(t, y[:, 1], 'b', label='$\dot x$')
ax.set_ylabel('$\dot x$ (m/s)')
ax = fig.add_subplot(413)
ax.grid()
ax.plot(t, y[:, 2], 'b', label='\\theta')
ax.set_ylabel('$\\theta$ (rad)')
ax = fig.add_subplot(414)
ax.grid()
ax.plot(t, y[:, 3], 'b', label='\dot \\theta')
ax.set_ylabel('$\dot \\theta$ (rad/s)')
ax.set_xlabel('time (sec)')
plt.show()

#
# --- LQ optimal control for the linearized system ---
#

Alq = A-B @ K
Blq = B
Clq = C
Dlq = D

syslqlinear = co.ss(Alq, Blq, Clq, Dlq)

# Define intial conditions
# The initial state for the linearized model is the deviation from the equilibrium point
x0 = np.array([0, 0, 0.1, 0])
X0 = x0

# define simulink parameters
T_final = 15
Ts = 0.0001
t = np.arange(0, T_final, Ts)

v = 0 * np.ones(np.shape(t))

[Y, T, X] = co.lsim(syslqlinear, v, t, X0)
# The actual state of physical system is state of the linearized system + the equilibrium point
X[:, 2] = X[:, 2] + np.pi

# Performance verification
plt.figure(figsize=(7, 7))
plt.subplot(4, 1, 1)
plt.plot(t, X[:, 0])
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$x_1$')
plt.title('Derived states of linearized system under LQ')
plt.subplot(4, 1, 2)
plt.plot(t, X[:, 1])
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$x_2$')
plt.subplot(4, 1, 3)
plt.plot(t, X[:, 2])
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$x_3$')
plt.subplot(4, 1, 4)
plt.plot(t, X[:, 3])
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$x_4$')
plt.savefig("lqlinearstates.pdf", format="pdf", bbox_inches="tight")
plt.show()

#
# ---  Observer design for the linearized system  ---
#

Ob = co.obsv(A, C)
rank = np.linalg.matrix_rank(Ob)
print('Tha rank of the observability matrix is ', rank)

# Calculate eigenvalue of the plant A matrix
EigA = np.linalg.eigvals(A)
print('The eigenvalues of the plant "A" matrix are: ', EigA)

# Calculate eigenvalues of the linearized system under LQ
EigCL = np.linalg.eigvals(A-np.dot(B, K))
print('The eigenvalues of the LQ system are: ', EigCL)

poles = np.array([-120, -122, -124, -126])
# Place the poles of the observer and calculate the observer gain matrix
L = co.place(A.T, C.T, poles).T
print('The observer gain matrix is: ', L)
# Check observer poles
est_poles = np.linalg.eigvals(A - np.dot(L, C))
print('Placed observer eigenvalues: ', est_poles)

# Simulation
# Define augmented system to run the simulation
Aaug = np.block([[A, np.zeros((4, 4))], [L @ C, A - L @ C]])
# Baug = np.concatenate((B, B), axis=0)
Baug = np.block([[B],[B]])
# Caug = np.concatenate((C, np.zeros((2, 4))), axis=1)
Caug = np.block([C,np.zeros((2, 4))])
Daug = np.array([[0], [0]])

sysobs = co.ss(Aaug, Baug, Caug, Daug)

# Define intial conditions
# Initial state is the deviation from the equilibrium point
x0 = np.array([0, 0, 0.1, 0])
xhat0 = np.array([0, 0, 0, 0])
X0 = np.array([x0, xhat0]).reshape((8, 1))

# Define simulink parameters
T_final = 15
t = np.arange(0, T_final, Ts)

# We focus just on state estimation and set the input zero first
v = 0 * np.ones(np.shape(t))

[Y, T, X] = co.lsim(sysobs, v, t, X0)
# Actual state is state of the linearized system + equilibrium point
X[:, 2] = X[:, 2] + np.pi
# Actual state is state of the linearized system + equilibrium point
X[:, 6] = X[:, 6] + np.pi

# Performance verification
plt.figure(figsize=(7, 7))
plt.subplot(4, 1, 1)
plt.plot(t, X[:, 0], t, X[:, 4], '--', linewidth=1.5)
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$x_1$')
plt.title('States (solid lines) and their estimates (dashed lines)')
plt.subplot(4, 1, 2)
plt.plot(t, X[:, 1], t, X[:, 5], '--', linewidth=1.5)
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$x_2$')
plt.subplot(4, 1, 3)
plt.plot(t, X[:, 2], t, X[:, 6], '--', linewidth=1.5)
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$x_3$')
plt.subplot(4, 1, 4)
plt.plot(t, X[:, 3], t, X[:, 7], '--', linewidth=1.5)
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$x_4$')
plt.savefig("obslinearstates.pdf", format="pdf", bbox_inches="tight")
plt.show()

#
# --- Observer State Feedback for the Linearized System ---
#

# Define augmented system to run the simulation
Acl = np.block([[A, -B @ K], [L @ C, A - L @ C - B @ K]])
# Bcl = np.concatenate((B, B), axis=0)
Bcl = np.block([[B],[B]])
# Ccl = np.concatenate((C, np.zeros((2, 4))), axis=1)
Ccl = np.block([C,np.zeros((2, 4))])
Dcl = np.array([[0], [0]])

syscl = co.ss(Acl, Bcl, Ccl, Dcl)

# Define intial conditions
x0 = np.array([0, 0, 0.1, 0])
xhat0 = np.array([0, 0, 0, 0])
X0 = np.array([x0, xhat0]).reshape((8, 1))

# Define simulink parameters
T_final = 15
t = np.arange(0, T_final, Ts)

# This time, let us even add some non-zero inputs
v = -100 * np.ones(np.shape(t))

[Y, T, X] = co.lsim(syscl, v, t, X0)
# Actual state is state of the linearized system + equilibrium point
X[:, 2] = X[:, 2] + np.pi
# Actual state is state of the linearized system + equilibrium point
X[:, 6] = X[:, 6] + np.pi

# Performance verification
plt.figure(figsize=(7, 7))
plt.subplot(4, 1, 1)
plt.plot(t, X[:, 0], t, X[:, 4], '--')
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$x_1$')
plt.title('States (solid lines) and their estimates (dashed lines))')
plt.subplot(4, 1, 2)
plt.plot(t, X[:, 1], t, X[:, 5], '--')
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$x_2$')
plt.subplot(4, 1, 3)
plt.plot(t, X[:, 2], t, X[:, 6], '--')
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$x_3$')
plt.subplot(4, 1, 4)
plt.plot(t, X[:, 3], t, X[:, 7], '--')
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$x_4$')
plt.savefig("lqobslinearstates.pdf", format="pdf", bbox_inches="tight")
plt.show()

plt.figure(figsize=(7, 7))
plt.subplot(4, 1, 1)
plt.plot(t, X[:, 0]-X[:, 4])
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$e_1$')
plt.title('State estimation errors')
plt.subplot(4, 1, 2)
plt.plot(t, X[:, 1]-X[:, 5])
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$e_2$')
plt.subplot(4, 1, 3)
plt.plot(t, X[:, 2]-X[:, 6])
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$e_3$')
plt.subplot(4, 1, 4)
plt.plot(t, X[:, 3]-X[:, 7])
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$e_4$')
plt.savefig("lqobslinearstateserr.pdf", format="pdf", bbox_inches="tight")
plt.show()

v = -100 * np.sin(10 * t)

[Y, T, X] = co.lsim(syscl, v, t, X0)
# Actual state is state of the linearized system + equilibrium point
X[:, 2] = X[:, 2] + np.pi
# Actual state is state of the linearized system + equilibrium point
X[:, 6] = X[:, 6] + np.pi

plt.figure(figsize=(7, 7))
plt.subplot(4, 1, 1)
plt.plot(t, X[:, 0], t, X[:, 4], '--')
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$x_1$')
plt.title('States (solid lines) and their estimates (dashed lines)')
plt.subplot(4, 1, 2)
plt.plot(t, X[:, 1], t, X[:, 5], '--')
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$x_2$')
plt.subplot(4, 1, 3)
plt.plot(t, X[:, 2], t, X[:, 6], '--')
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$x_3$')
plt.subplot(4, 1, 4)
plt.plot(t, X[:, 3], t, X[:, 7], '--')
plt.xlabel('time (sec)')
plt.grid()
plt.ylabel('$x_4$')
plt.savefig("lqobslinearstatessininput.pdf", format="pdf", bbox_inches="tight")
plt.show()
