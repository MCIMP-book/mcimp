% ssdescription/msd.m
m = 1;
k = 2;
b = 1;
A = [0 1; -k/m -b/m];
B = [0; 1/m];
C = [1 0];
D = 0;
sys = ss(A,B,C,D) % state space representation
[num,den] = ss2tf(A,B,C,D); % the function ss2tf provides the transfer function of a state-space model
sys_tf = tf(num,den)
