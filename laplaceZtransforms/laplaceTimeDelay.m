% laplaceZtransforms/laplaceTimeDelay.m
syms t
d = dirac(t-4);
D = laplace(d)

D =
exp(-4*s)
