# probability_review/sum_2rv.py
import numpy as np
import matplotlib.pyplot as plt

# Generate three random arrays of size 1e5
X1 = np.random.rand(1, int(1e5))
X2 = np.random.rand(1, int(1e5))
X3 = np.random.rand(1, int(1e5))

# Compute the sum of X1 and X2
Z = X1 + X2

# Compute the histogram of Z with 100 bins
fz, x = np.histogram(Z, bins=100)

# Normalize the histogram by the area and the bin width
w_fz = x[-1] / len(fz)
fz = fz / np.sum(fz) / w_fz

# Plot the histogram as a bar chart
plt.bar(x[:-1], fz, width=w_fz)
plt.xlabel('$x$')
plt.ylabel('$p_Z(x)$')
plt.show()
