# laplaceZtransforms/partial_fraction_expansion.py
import sympy
s = sympy.symbols('s')
G = 32/s/(s+4)/(s+8)
print(sympy.apart(G))
