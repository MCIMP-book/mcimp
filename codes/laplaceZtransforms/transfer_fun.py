# laplaceZtransforms/transfer_fun.py
import control as ct
import matplotlib.pyplot as plt
import numpy as np
# Creating a transfer function system

num = [1,2] # Numerator co-efficients
den = [1,2,3] # Denominator co-efficients

sys_tf = ct.tf(num,den)
print(sys_tf)

# Poles and zeros
poles = ct.poles(sys_tf)
zeros = ct.zeros(sys_tf)
print('\nSystem Poles = ', poles, '\nSystem Zeros = ', zeros)

T,yout = ct.step_response(sys_tf)

# Plot the response
plt.figure(1,figsize = (6,4))
plt.plot(T,yout)

plt.grid(True)
plt.ylabel("y")
plt.xlabel("Time (sec)")
plt.show()

T,yout_i = ct.impulse_response(sys_tf)

# Plot the response
plt.figure(1,figsize = (6,4))
plt.plot(T,yout_i)

plt.grid(True)
plt.ylabel("y")
plt.xlabel("Time (sec)")
plt.show()

u1 = np.full((1,len(T)),2) #Create an array of 2's, equal to 2*step
u2 = np.sin(T)

T,yout_u1 = ct.forced_response(sys_tf,T,u1) # Response to input 1
T,yout_u2 = ct.forced_response(sys_tf,T,u2) # Response to input 2

plt.figure(2,figsize = (6,4))

plt.plot(T,yout_u1)
plt.plot(T,yout_u2)

plt.grid()
plt.xlabel("Time (sec)")
plt.ylabel("y")
plt.legend(["Input 1","Input 2 (sin)"])
plt.show()
