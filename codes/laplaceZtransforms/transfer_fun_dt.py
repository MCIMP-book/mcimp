# laplaceZtransforms/transfer_fun_dt.py
import control as ct
import matplotlib.pyplot as plt
import numpy as np
Ts = 0.1 # sampling time

num = [0.09952, -0.08144] # Numerator co-efficients
den = [1, -1.792, 0.8187] # Denominator co-efficients

sys_tf = ct.tf(num,den, Ts)
print(sys_tf)

poles = ct.poles(sys_tf)
zeros = ct.zeros(sys_tf)
print('\nSystem Poles = ', poles, '\nSystem Zeros = ', zeros)

T,yout = ct.step_response(sys_tf)

# Plot the response
plt.figure(1,figsize = (6,4))
# instead of plt.plot(T,yout), we use step to show the discrete nature of the response
# plt.step(T,yout)
# at the time of writing this code, there is a difference between how MATLAB and Python make stair plots
# to correctly show the initial one-step delay, we need to append a zero at the beginning of the output
plt.step(T,np.append(0,yout[0:-1]))

plt.grid(True)
plt.ylabel("y")
plt.xlabel("Time (sec)")
plt.show()

T,yout_i = ct.impulse_response(sys_tf)

# Plot the response
plt.figure(1,figsize = (6,4))
plt.step(T,np.append(0,yout_i[0:-1]))

plt.grid(True)
plt.ylabel("y")
plt.xlabel("Time (sec)")
plt.show()

u1 = np.full((1,len(T)),2) #Create an array of 2's, equal to 2*step
u2 = np.sin(T)

T,yout_u1 = ct.forced_response(sys_tf,T,u1) # Response to input 1
T,yout_u2 = ct.forced_response(sys_tf,T,u2) # Response to input 2

plt.figure(2,figsize = (6,4))

plt.step(T,np.append(0,yout_u1[0:-1]))
plt.step(T,np.append(0,yout_u2[0:-1]))

plt.grid()
plt.xlabel("Time (sec)")
plt.ylabel("y")
plt.legend(["Input 1","Input 2 (sin)"])
plt.show()
