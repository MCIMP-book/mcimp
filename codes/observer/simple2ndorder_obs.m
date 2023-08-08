% observer/simple2ndorder_obs.m
% plant model
A = [0 1;-4 -0.2];
B = [0 1]';
C = [1 0];
sys = ss(A,B,C,0);

eig(A)

L = place(A',C',[-2,-3])'

eig(A-L*C)
