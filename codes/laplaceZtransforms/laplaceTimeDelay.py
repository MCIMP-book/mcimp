# laplaceZtransforms/laplaceTimeDelay.py
import sympy
t, s = sympy.symbols('t, s')
d = sympy.DiracDelta(t-4)
D = sympy.laplace_transform(d, t, s, noconds=True)
print(D)
