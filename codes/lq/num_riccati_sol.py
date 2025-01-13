# lq/num_riccati_sol.py
# Linear quadratic optimal control of a double integrator
# Running environment: Python 3.8.5
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import numpy as np

# system dynamics


def vdp1(T, P): return [1-1/R*P[1]**2,
                        P[0] - 1/R*P[1]*P[2],
                        2*P[1] - 1/R*P[2]**2]


R = 0.0001

sol = solve_ivp(vdp1, [0, 20], [1, 0, 1],
                dense_output=True, t_eval=np.linspace(0, 1.5, 100))

T = sol.t
P = sol.y

plt.plot(T, P[0], T, P[1], '--', T, P[2], ':')
plt.legend(['$p^*_{11}$', '$p^*_{12}$', '$p^*_{22}$'])
plt.title('$P^*$ with $R=0.0001$')
plt.xlim(0)
plt.xlabel('time/s')
plt.savefig('Rsmall.pdf')
plt.show()

R = 1

sol = solve_ivp(vdp1, [0, 20], [1, 0, 1],
                dense_output=True, t_eval=np.linspace(0, 15, 1000))

T = sol.t
P = sol.y

plt.plot(T, P[0], T, P[1], '--', T, P[2], ':')
plt.legend(['$p^*_{11}$', '$p^*_{12}$', '$p^*_{22}$'])
plt.title('$P^*$ with $R=1$')
plt.xlim(0)
plt.xlabel('time/s')
plt.savefig('Rmid.pdf')
plt.show()

R = 100

sol = solve_ivp(vdp1, [0, 40], [1, 0, 1],
                dense_output=True, t_eval=np.linspace(0, 40, 10000))

T = sol.t
P = sol.y

plt.plot(T, P[0], T, P[1], '--', T, P[2], ':')
plt.legend(['$p^*_{11}$', '$p^*_{12}$', '$p^*_{22}$'])
plt.title('$P^*$ with $R=100$')
plt.xlim(0)
plt.xlabel('time/s')
plt.savefig('Rlarge.pdf')
plt.show()

# different initial value
sol = solve_ivp(vdp1, [0, 40], [20, 0, 2],
                dense_output=True, t_eval=np.linspace(0, 40, 10000))

T = sol.t
P = sol.y

plt.plot(T, P[0], T, P[1], '--', T, P[2], ':')
plt.legend(['$p^*_{11}$', '$p^*_{12}$', '$p^*_{22}$'])
plt.title('$P^*$ with $R=100$ and a different initial value')
plt.xlim(0)
plt.xlabel('time/s')
plt.savefig('Rlarge_different_init.pdf')
plt.show()
