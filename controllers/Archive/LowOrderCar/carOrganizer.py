import numpy as np
"Allows for simplification of code for generating string names for Webots car simulation"

def createLocationString(lateral, longitudinal):
    R = ""
    if (abs(lateral) != 1 & abs(longitudinal) != 1):
        raise Exception("Use only 1 or negative 1 for locations")

   
    if (longitudinal < 0):
        R = R+"Rear"
    else:
        R = R+"Front"
    if (lateral < 0):
        R = R+"Right"
    else:
        R = R+"Left"
    return R 

def calculateDiffRatio(wheelbase,steeringAngle,trackwidth):
    if(steeringAngle == 0):
        return 0.5
    rightRadius = np.abs(wheelbase/(np.tan(steeringAngle)-trackwidth/2))
    leftRadius = wheelbase/(np.tan(steeringAngle)+trackwidth/2)

    return rightRadius/(leftRadius+rightRadius)

#Takes differential ratio calculated with other helper function and spits out left and right torques
def simulateDifferential(inputTorque, ratio):
    rightTorque = inputTorque*ratio
    leftTorque = inputTorque*(1-ratio)
    return [[leftTorque,rightTorque]]