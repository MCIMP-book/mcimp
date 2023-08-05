% lyapunov/dtlyapunov_sol.m
A=[ 0 1 0; 0 0 1; 0.275 -0.225 -0.1]
Q = eye(3)
P = dlyap(A',Q)
eig(P)
