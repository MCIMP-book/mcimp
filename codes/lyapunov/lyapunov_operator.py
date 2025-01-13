# lyapunov/lyapunov_operator.py
import numpy as np
A = [[-1,1],[-1,0]]; I2=np.eye(2); AT=np.transpose(A)
L_A=np.kron(I2,AT)+np.kron(AT,I2)
eigLA=np.linalg.eigvals(L_A)
eigA=np.linalg.eigvals(A)
print('Eigenvalues of L_A:',eigLA)
print('Eigenvalues of A:',eigA)
