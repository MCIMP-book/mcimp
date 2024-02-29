# lyapunov/ctlyapunov_sol.py
import control as ct
import numpy as np
A = np.array([[-1,1],[-1,0]])
Q = np.identity(2)
P = ct.lyap(A.transpose(),Q)
print(P)
w = np.linalg.eigvals(P)# compute eigenvalues of P
print(w)
