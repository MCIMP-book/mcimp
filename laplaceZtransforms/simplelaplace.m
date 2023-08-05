% laplaceZtransforms/simplelaplace.m
syms a t
% exponential functions
f = exp(-a*t);
F = laplace(f)

F =
1/(a + s)

g = exp(-2*t);
G = laplace(g)

G =
1/(s + 2)
% ramp function
h = 2*t;
H = laplace(h)

H =
2/s^2
% impulse function
d = dirac(t);
D = laplace(d)

D =
1
