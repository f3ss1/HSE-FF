import numpy as np
from matplotlib import pyplot as plt

a = np.genfromtxt('1.txt')

plt.figure(figsize=(20, 5))
plt.plot(a)
plt.grid()
plt.show()

