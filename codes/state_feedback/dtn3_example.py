# state_feedback/dtn3_example.py
import control as ct
import numpy as np
A = np.array([[1,1,-2],[0,1,1],[0,0,1]])
B = np.array([[1],[0],[1]])
p = [0,0.1,0.2]
K = ct.place(A, B, p)
print(K)
