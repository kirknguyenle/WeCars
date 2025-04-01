
import matplotlib.pyplot as mpl
import numpy as np
import carOrganizer as cO

Speed = 60 #Km/h
angles = np.arange(-1.57/4,1.57/4, 0.025)
wheelbase = 6 # meters
trackwidth = 2 # meters
leftTorque = []
rightTorque = []
ratios = []

for i in range(len(angles)):
    ratio =  cO.calculateDiffRatio(wheelbase,angles[i],trackwidth)
    ratios.append(ratio)
    torques = cO.simulateOpenDifferential(Speed,ratio)
    leftTorque.append(torques[0])
    rightTorque.append(torques[1])

mpl.plot(angles, ratios, label='Left Speed')
#mpl.plot(angles, np.abs(rightTorque), label='Right Speed')
mpl.xlabel('Steering Angle')
mpl.ylabel('ratio')
mpl.legend()
mpl.show()