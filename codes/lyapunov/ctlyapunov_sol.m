% lyapunov/ctlyapunov_sol.m
A = [-1,1;-1,0]
Q = eye(2)
P = lyap(A',Q)
w = eig(P)
