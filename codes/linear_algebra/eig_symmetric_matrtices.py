# linear_algebra/eig_symmetric_matrtices.py
import numpy as np
N = 8 # Define matrix size
# Create a random matrix P with values between -200 and 200 with size N x N
P = np.random.randint(-200, 200, size=(N, N))
# Make P symmetric by averaging it with its transpose
P_symm = (P + P.T) / 2
# Compute the eigenvalues of the symmetric matrix P_symm
lambdas = np.linalg.eigvals(P_symm)
print(lambdas)
