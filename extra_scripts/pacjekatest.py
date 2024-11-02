import matplotlib.pyplot as mpl
import numpy as np



D = 20
C = 1.4
B = 0.714
E = 0.2
K = C*B*D

angles = np.arange(0, 20,0.05)
linear = []
pacjeka = []
webots = []

for i in range (len(angles)):
    if K*angles[i] < 20:
        linear.append(K*(angles[i]))
    else: 
        linear.append(20)
    pacjeka.append(D*np.sin(C*np.arctan(B*(angles[i])-E*(B*(angles[i])-np.arctan(B*(angles[i]))))))

mpl.plot(angles, linear, label='Linear')
mpl.plot(angles, pacjeka, label='Pacjeka')
mpl.xlabel('Slip Angle (degrees)')
mpl.ylabel('Lateral Force (N)')
mpl.title('Comparison of Linear and Pacjeka Models')
mpl.legend()
mpl.show()