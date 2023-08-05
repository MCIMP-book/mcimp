% laplaceZtransforms/CTDCgain.m
s = tf('s');
G = (2*s+3)/(4*s^2+3*s+1);
dcgain(G)
