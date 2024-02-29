# sssolution/number_e.py
# Python code illustrating the convergence of the infinite series to the irrational number e
import math
import matplotlib.pyplot as plt
print(math.e)
N = 10
eapprox = [sum(1/math.factorial(k) for k in range(n)) for n in range(1,N+1)]

print(eapprox)

plt.figure()
plt.scatter([i for i in range(N)], eapprox, color='r')
plt.plot(range(N), [math.e]*N, 'b.')
plt.xlim(0, N)
plt.ylim(0, 3)
plt.xlabel('$n$')
plt.xticks(range(N+1))
plt.ylabel('Convergence of $\sum_{n=0}^{\infty}{1}/{n!}$')
