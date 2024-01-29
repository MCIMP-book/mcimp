# ssdescription/vehicle_steer_linear.py
import numpy as np
import matplotlib.pyplot as plt
import control.matlab as ct
from scipy.integrate import odeint

# Vehicle steering: bicycle model
# Accuracy of linearization
a = 1.799/2  # Length from rear wheels to center of mass
b = 1.799  # Length of the car
v_0 = 10  # Initial velocity in meters per second

# Define the nonlinear system

def f(x, t, u):
    y = x[0]
    theta = x[1]
    dydt = v_0*np.sin(np.arctan(a*np.tan(u)/b)+theta)
    dthetadt = v_0*np.sin(np.arctan(a*np.tan(u)/b))/a
    dxdt = [dydt, dthetadt]
    return dxdt

def h(x, u):
    return x

# Set simulation time and initial conditions
tspan = [0, 10]
x0 = [0, 0]

# Define the state-space model of the vehicle with the output being only the position
A = np.array([[0, v_0], [0, 0]])
B = np.array([a*v_0/b, v_0/b]).reshape(-1, 1)
C = np.array([1, 0])
D = np.array([0])

## Effect of varying input magnitudes
# Simulate the nonlinear system using odeint solver
Ugain = [0.05, 0.1, 0.5, 1]
t = np.linspace(tspan[0], tspan[1], 1000)
Y = np.zeros((len(t), len(Ugain)))
Ylinear = Y.copy()
for ii in range(len(Ugain)):
        # Define input signal
        # Sinusoidal input signal with a frequency of 1 Hz and an amplitude of 0.1 radians
    u = Ugain[ii]*np.sin(2*np.pi*1*t)
    for jj in range(len(t)):
        # Simulate the nonlinear system using odeint solver
        x = odeint(f, x0, [t[jj-1],t[jj]], args=(u[jj],))
        x0 = x[1]
        Y[jj, ii] = x[1][0]

    # Simulate the linearized system
    x0 = [0, 0]
    Ylinear[:, ii], _, _ = ct.lsim(ct.ss(A, B, C, D), u, t, x0)
    # or use ct.forced_response from plain control instead of control.matlab

# Plot the results
plt.figure()
plt.subplot(411)
plt.plot(t, Y[:, 0], t, Ylinear[:, 0], 'r--')
plt.title('Response to input at 1 Hz and magnitude 0.05 radians')
plt.ylabel('Position y (m)')
plt.legend(['nonlinear model', 'linearized model'], loc='upper left')
plt.subplot(412)
plt.plot(t, Y[:, 1], t, Ylinear[:, 1], 'r--')
plt.title('Response to input at 1 Hz and magnitude 0.1 radians')
plt.ylabel('Position y (m)')
plt.legend(['nonlinear model', 'linearized model'], loc='upper left')
plt.subplot(413)
plt.plot(t, Y[:, 2], t, Ylinear[:, 2], 'r--')
plt.title('Response to input at 1 Hz and magnitude 0.5 radians')
plt.ylabel('Position y (m)')
plt.legend(['nonlinear model', 'linearized model'], loc='upper left')
plt.subplot(414)
plt.plot(t, Y[:, 3], t, Ylinear[:, 3], 'r--')
plt.title('Response to input at 1 Hz and magnitude 1 radians')
plt.ylabel('Position y (m)')
plt.legend(['nonlinear model', 'linearized model'], loc='upper left')
plt.xlabel('Time (s)')
plt.show()

## Effect of varying input frequencies
# Simulate the nonlinear system using odeint solver

FreqVect = [1,5,10,30]
t = np.linspace(tspan[0],tspan[1],1000)
Y = np.zeros((len(t),len(FreqVect)))
Ylinear = Y.copy()
for ii in range(len(FreqVect)):
    # Define input signal
    u = 0.05*np.sin(2*np.pi*FreqVect[ii]*t)
    # Simulate the nonlinear system using odeint solver
    for jj in range(len(t)):
        # Simulate the nonlinear system using odeint solver
        x = odeint(f, x0, [t[jj-1],t[jj]], args=(u[jj],))
        x0 = x[1]
        Y[jj, ii] = x[1][0]

    # Simulate the linearized system
    x0 = [0, 0]
    Ylinear[:, ii], _, _ = ct.lsim(ct.ss(A, B, C, D), u, t, x0)
    # exercise: use control.forced_response instead to obtain the equivalent result
