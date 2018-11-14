from scipy.interpolate import interp1d
import numpy as np
import matplotlib.pyplot as plt

x = np.array([0, 1, 15,  50])
y = np.array([4, 4, 3, 0])

print(x, y)

f_line = interp1d(x, y)
f_CS = interp1d(x, y, kind='cubic')

xnew = np.linspace(0, 50, num=50)
plt.plot(x, y, 'o', xnew, f_CS(xnew), '-')
plt.legend(['Raw data', 'Lagrange', 'Cubic spline'], loc='best')
plt.xlim([0, 50])
plt.ylim([0, 4])
plt.show()
