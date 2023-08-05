# controllability_observability/connectedsys.py
import control
import numpy as np

A1 = [[0, 1], [-2, -3]]
B1 = [[0], [1]]
C1 = [[2, 1]]
sys1 = control.ss(A1, B1, C1, 0)
P1 = control.ctrb(A1,B1)
print(np.linalg.matrix_rank(P1))
Q1 = control.obsv(A1,C1)
print(np.linalg.matrix_rank(Q1))

A2 = [[-1, 0], [0, -3]]
B2 = [[1], [1]]
C2 = [[1, 1]]
sys2 = control.ss(A2, B2, C2, 0)
P2 = control.ctrb(A2,B2)
print(np.linalg.matrix_rank(P2))
Q2 = control.obsv(A2,C2)
print(np.linalg.matrix_rank(Q2))

sys_s = control.series(sys1, sys2)
sys_p = control.parallel(sys1, sys2)

Ps = control.ctrb(sys_s.A, sys_s.B)
print(np.linalg.matrix_rank(Ps))
Qs = control.obsv(sys_s.A, sys_s.C)
print(np.linalg.matrix_rank(Qs))

Pp = control.ctrb(sys_p.A, sys_p.B)
print(np.linalg.matrix_rank(Pp))
Qp = control.obsv(sys_p.A, sys_p.C)
print(np.linalg.matrix_rank(Qp))
