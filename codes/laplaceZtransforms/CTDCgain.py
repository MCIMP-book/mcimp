# laplaceZtransforms/CTDCgain.py
import control as ct
s = ct.tf('s')
G = (2*s+3)/(4*s**2+3*s+1);
print(ct.dcgain(G))
