# laplaceZtransforms/timescaledGeometricSeqZ.py
import lcapy as lc
from lcapy.discretetime import n
f=0.5**n
print(f)
F = f.ZT()
print(F)

f1 = n*f
print(f1)
F1 = f1.ZT()
print(F1)
