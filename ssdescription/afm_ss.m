% ssdescription/afm_ss.m
m1 = 8e-14;
m2 = 3e-13;
b1 = 2.19e-8;
b2 = 9.43e-10;
k1 = 3e-2;
k2 = 7e-3;
A = [0 0 1 0; 0 0 0 1; -k1/m1 k1/m1 -b1/m1 b1/m1; k1/m2 -(k1+k2)/m2 b1/m2 -(b1+b2)/m2];
B = [0; 0; 1/m1; -1/m2];
C = [1 0 0 0];
D = 0;
sys = ss(A,B,C,D)
[num,den] = ss2tf(A,B,C,D);
sys_tf = tf(num,den)
figure, pzmap(sys_tf)
figure, bodeplot(sys_tf)
