% controllability_observability/controllability_matrix.m
A = [0.4 0.4 0 0; -0.9 -0.07 0 0; 0 0 0.4 0.4; 0 0 -0.9 -0.07];
B = [0.3 0.4 0.3 0.4]';
P = ctrb(A,B);
rank(P)
