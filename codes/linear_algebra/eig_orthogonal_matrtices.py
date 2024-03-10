# linear_algebra/eig_orthogonal_matrtices.py
import numpy as np
from scipy.linalg import qr
n = 3
H = np.random.randn(n, n)
Q, _ = qr(H)
print (np.dot(Q,Q.T))
print (np.dot(Q.T,Q))
eigQ = np.linalg.eigvals(Q)
print(np.abs(eigQ))
