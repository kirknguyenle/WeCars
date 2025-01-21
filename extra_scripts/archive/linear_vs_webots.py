import matplotlib.pyplot as mpl
import numpy as np

k = 3 #N/rad
s = 3 #m/s/N
v_x = 40 #m/s

angles = np.arange(1, 30)
linear = []
webots = []
print(angles)
for i in range(len(angles)):
    linear.append(k*np.radians(i))
    webots.append(((np.tan(np.radians(i))*v_x)/(s)))
print(linear)

mpl.plot(angles, linear, label='Linear')
mpl.plot(angles, webots, label='Webots')
mpl.xlabel('Slip Angle (degrees)')
mpl.ylabel('Force (N)')
mpl.legend()
mpl.show()