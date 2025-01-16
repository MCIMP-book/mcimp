% c2d/dteigenvalue.m
m = 1;
dt = 0.1;
A = [0 1;0 0]; B = [0;1]; C = [1/m 0]; D = 0;

G_s = ss(A,B,C,D);
G_z = c2d(G_s, dt,'zoh');

G_z.A

% eigenvalues of continuous-time system
eig(A)
% eigenvalues of discretized system
eig(G_z.A)
