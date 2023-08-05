# lyapunov/dtlyapunov_sol.py
import control as co
import numpy as np
from numpy.linalg import eig
A = np.array([[0,1,0],[0,0,1],[0.275,-0.225,-0.1]])
Q = np.identity(3)
P = co.dlyap(A.transpose(),Q)
w,v = eig(P) # compute eigenvalue and eigenvectors of P
print(w) # print the eigenvalues
