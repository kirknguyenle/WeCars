import matplotlib.pyplot as mpl
import numpy as np
import math 

v_x = 26.8224 #m/s

D = 20
C = 1.4
B = 0.714
E = 0.2
K = C*B*D

angles = np.arange(0, 1.57, 0.05)
print(angles)
linear = []
webots = []
pacjeka = []
print(angles)
for i in range(len(angles)):
    if K*angles[i] < 20:
        linear.append(K*angles[i])
    else: 
        linear.append(20)
    if (np.tan(angles[i])*v_x)/(K) < 20:
        webots.append(((np.tan((angles[i]))*v_x)/(K)))
    else:
        webots.append(20)
    pacjeka.append(D*np.sin(C*np.arctan(B*(angles[i])-E*(B*(angles[i])-np.arctan(B*(angles[i]))))))
    
print(linear)

mpl.plot(angles, linear, label='Linear')
mpl.plot(angles, webots, label='Webots')
mpl.plot(angles, pacjeka, label='Pacjeka')
mpl.xlabel('Slip Angle (Radians)')
mpl.ylabel('Force (N)')
mpl.legend()
mpl.show()