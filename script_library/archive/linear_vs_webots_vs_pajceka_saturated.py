import matplotlib.pyplot as mpl
import numpy as np
import math 

v_x = 20 #m/s
v_x2 = 40
D = 10000
C = 1.4
B = 0.714
E = 0.2
K = C*B*D

angles = np.arange(-1.57, 1.57, 0.05)
print(angles)
linear = []
pacjeka = []
print(angles)
for i in range(len(angles)):
    if 0<K*angles[i] < 9000:
        linear.append(K*angles[i])
    elif -9000 < K*angles[i] <0: 
        linear.append(K*angles[i])
    elif K*angles[i] > 9000:
        linear.append(9000)
    elif K*angles[i] < -9000:
        linear.append(-9000)
    pacjeka.append(D*np.sin(C*np.arctan(B*(angles[i])-E*(B*(angles[i])-np.arctan(B*(angles[i]))))))
    
print(linear)

mpl.plot(angles, linear, label='Linear')
mpl.plot(angles, pacjeka, label='Pacjeka')
mpl.xlabel('Slip Angle (Radians)')
mpl.ylabel('Force (N)')
mpl.legend()
mpl.show()