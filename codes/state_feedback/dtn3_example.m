% state_feedback/dtn3_example.m
A = [1,1,-2;0,1,1;0,0,1];
B = [1;0;1];
p = [0;0.1;0.2];
K = place(A, B, p)
