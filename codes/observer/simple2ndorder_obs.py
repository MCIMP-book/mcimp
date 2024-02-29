# observer/simple2ndorder_obs.py
import control as ct
import numpy as np
A = np.array([[0, 1],[-4, -0.2]])
C = np.array([[1], [0]]).T
L = ct.place(A.T,C.T,[-2, -3]).T
print(L)
