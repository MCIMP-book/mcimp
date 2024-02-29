# laplaceZtransforms/CTDCgain_caution.py
import control as ct
H = ct.tf([0, 3],[1, -2])
print(ct.dcgain(H))
T, yout = ct.step_response(H)
print(yout)
