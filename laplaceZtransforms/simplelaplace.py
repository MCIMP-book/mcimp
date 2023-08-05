# laplaceZtransforms/simplelaplace.py
import sympy
t, s = sympy.symbols('t, s')
a = sympy.symbols('a', real=True, positive=True)
f = sympy.exp(-a*t)
F = sympy.laplace_transform(f, t, s, noconds=True)
print(F)

g = sympy.exp(-2*t)
G = sympy.laplace_transform(g, t, s, noconds=True)
print(G)

h = 2*t
H = sympy.laplace_transform(h, t, s, noconds=True)
print(H)

d = sympy.DiracDelta(t)
D = sympy.laplace_transform(d, t, s, noconds=True)
print(D)
