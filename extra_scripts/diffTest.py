
import matplotlib.pyplot as mpl
import numpy as np
import carOrganizer as cO

Torque = 300 #N*M 
angles = np.arange(0,1.57/4, 0.025)
print (angles)
wheelbase = 6 # meters
trackwidth = 2 # meters
leftTorque = []
rightTorque = []

for i in range(len(angles)):
    ratio =  cO.calculateDiffRatio(wheelbase,angles[i],trackwidth)
    print(ratio)
    torques = cO.simulateDifferential(Torque,ratio)
    leftTorque.append(torques[0][0])
    rightTorque.append(torques[0][1])

mpl.plot(angles, np.abs(leftTorque), label='left Torque')
mpl.plot(angles, np.abs(rightTorque), label='right Torque')
mpl.xlabel('Steering Angle')
mpl.ylabel('Torque N*M')
mpl.legend()
mpl.show()