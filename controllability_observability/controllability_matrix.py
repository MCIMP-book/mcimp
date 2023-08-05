# controllability_observability/controllability_matrix.py
import numpy as np
import control as co
A = np.array([[0.4, 0.4, 0, 0], [-0.9, -0.07, 0, 0], [0, 0, 0.4, 0.4], [0, 0, -0.9, -0.07]])
B = np.array([[0.3], [0.4], [0.3], [0.4]])
P = co.ctrb(A,B)
rankP = np.linalg.matrix_rank(P)
print(rankP)
