% laplaceZtransforms/timescaledGeometricSeqZ.m
syms k;
f = 0.5^k;
F = ztrans(f)
f1 = k*f;
F1 = ztrans(f1)
