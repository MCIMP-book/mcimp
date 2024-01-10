# laplaceZtransforms/CTDCgain_caution.py
import control as co
H = co.tf([0, 3],[1, -2])
print(co.dcgain(H))
T, yout = co.step_response(H)
print(yout)
