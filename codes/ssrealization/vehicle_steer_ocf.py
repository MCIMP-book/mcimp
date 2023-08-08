# ssrealization/vehicle_steer_ocf.py
import numpy as np
import control as ct

a = 1.799/2 # Length from rear wheels to center of mass
b = 1.799 # Length of the car
v_0 = 10 # Initial velocity in meters per second

# Linearized System
# Define the state-space model of the vehicle
A = np.array([[0, v_0], [0, 0]])
B = np.array([a*v_0/b, v_0/b])
C = np.array([[1, 0], [0, 1]])
D = np.array([[0], [0]])

sys = ct.StateSpace(A, B, C, D)

# Choosing y as the single output
C1 = np.array([[1, 0]])
D1 = np.array([0])
sys_y = ct.StateSpace(A, B, C1, D1)

# Transfer function by analysis
Gy = v_0/b*ct.TransferFunction([0, a, v_0], [1, 0, 0])
print(Gy)

# Computed transfer function
G_y = ct.ss2tf(sys_y.A, sys_y.B, sys_y.C, sys_y.D)
print(G_y)
