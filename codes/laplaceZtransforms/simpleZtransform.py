# laplaceZtransforms/simpleZtransform.py
import lcapy as lc
from lcapy.discretetime import n
import sympy
a = sympy.symbols('a')
f = a**n
F = f.ZT()
print(F)
