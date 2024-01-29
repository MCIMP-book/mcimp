# kalman_decompose/uncontrollablesys.py
import numpy as np
import control as co
from scipy.linalg import orth
from scipy.linalg import null_space
b = 1
m = 1
k1 = 0.5
k2 = 1
A = np.array([[-b/m, -1/m, -1/m], [k1, 0, 0], [k2, 0, 0]])
B = np.array([[1/m], [0], [0]])
P = co.ctrb(A,B)
Mc = orth(P)
Muc = null_space(P.transpose())
M = np.column_stack((Mc,Muc))
print('M: ')
print(M)
tildeA = (np.linalg.inv(M)@A)@M
print('tilde A:')
print(tildeA)
tildeB = np.linalg.inv(M)@B
print('tilde B:')
print(tildeB)
