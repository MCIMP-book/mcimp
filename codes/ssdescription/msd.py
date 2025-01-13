# ssdescription/msd.py
import control as ct
import numpy as np
m = 1
k = 2
b = 1
A = np.array([[0,1],[-k/m,-b/m]])
B = np.array([[0], [1/m]])
C = np.array([1,0])
D = np.array([0])
sys = ct.ss(A,B,C,D) # state space representation
print(sys)
sys_tf = ct.ss2tf(sys)
print(sys_tf)
