# observer/simple2ndorder_obs.py
import control as co
import numpy as np
A = np.array([[0, 1],[-4, -0.2]])
C = np.array([[1], [0]]).T
L = co.place(A.T,C.T,[-2, -3]).T
print(L)
