# linear_algebra/eig_skew_symmetric_matrtices.py
import numpy as np
N = 10
P = np.random.randint(-200,200,size=(N,N))
P_symm = (P - P.T)/2
lambdas, _ = np.linalg.eig(P_symm)
print(lambdas)
