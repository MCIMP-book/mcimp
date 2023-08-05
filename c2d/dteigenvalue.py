# c2d/dteigenvalue.py
# Discretization of continuous-time state-space systems in Python
import control
import numpy

m = 1
dt = 0.1
A = [[0, 1], [0, 0]]
B = [[0], [1]]
C = [[1/m, 0]]
D = 0

G_s = control.ss(A, B, C, D)
G_z = control.c2d(G_s, dt, 'zoh')

print(G_z.A)

# eigenvalues of continuous-time system
eigA, eigvecA = numpy.linalg.eig(A)
print(eigA)

# eigenvalues of discretized system
eigAd, eigvecAd = numpy.linalg.eig(G_z.A)
print(eigAd)
