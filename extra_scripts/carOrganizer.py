import numpy as np
"Allows for simplification of code for generating string names for Webots car simulation"

def createLocationString(lateral, longitudinal):
    R = ""
    if (abs(lateral) != 1 & abs(longitudinal) != 1):
        raise Exception("Use only 1 or negative 1 for locations")

    if (lateral < 0):
        R = R+"right"
    else:
        R = R+"left"
    if (longitudinal < 0):
        R = R+"rear"
    else:
        R = R+"front"
    return R 

    

def calculateDiffRatio(wheelbase,steeringAngle,trackwidth):
    if(steeringAngle == 0):
        return 0.5
    rightRadius =wheelbase/(np.tan(steeringAngle))-trackwidth/2
    leftRadius = wheelbase/(np.tan(steeringAngle))+trackwidth/2

    return rightRadius/(leftRadius+rightRadius)

#Takes the right and left "resistances" and creates a torque bias ratio. 
def perfectTorqueVector(leftI, rightI):
    return rightI/(leftI+rightI)



#Takes differential ratio calculated with other helper function and spits out left and right torques
def simulatePerfectDifferential(inputTorque, ratio):
    rightTorque = inputTorque*ratio
    leftTorque = inputTorque*(1-ratio)
    return [leftTorque,rightTorque]



def simulateOpenDifferential(inputSpeed, ratio):
    rightSpeed = inputSpeed*ratio
    leftSpeed = inputSpeed*(1-ratio)
    return [leftSpeed,rightSpeed]

def mirrorY(vec):
    u = []
    u[0] = vec[0]
    u[1] = -vec[1]
    u[2] = vec[2]
    return u 
