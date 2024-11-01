import matplotlib.pyplot as mpl
import numpy as np

k = 20 #N/deg

D = 20
C = 1.4
B = 0.714
E = 0.2

angles = np.arange(1, 90,0.05)
linear = []
pacjeka = []

for i in range (len(angles)):
    linear.append(k*(angles[i]))
    pacjeka.append(D*np.sin(C*np.arctan(B*(angles[i])-E*(B*(angles[i])-np.arctan(B*(angles[i]))))))

#mpl.plot(angles, linear, label='Linear')
mpl.plot(angles, pacjeka, label='Pacjeka')
mpl.xlabel('Slip Angle (degrees)')
mpl.ylabel('Lateral Force (N)')
mpl.title('Comparison of Linear and Pacjeka Models')
mpl.legend()
mpl.show()