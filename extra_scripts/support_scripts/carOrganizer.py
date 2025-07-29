import numpy as np


#Allows for simplification of code for generating string names for Webots car simulation
#Location indecies are 0 for left, and 
def createLocationString(lateral, longitudinal):
    R = ""
    if (longitudinal < 0.99):
        R = R+"rear"
    else:
        R = R+"front"
    if (lateral > 0.01):
        R = R+"right"
    else:
        R = R+"left"
    return R 

#Uses turn radius wheel-speed fraction from left-right (lever rule)
def radiusDiffRatio(trackwidth, radius, direction):
    if(radius > 200):
        return 0.5
    else:
        rightRadius = radius+(trackwidth/2*-1*direction)
        leftRadius = radius + (trackwidth/2*direction)
    return rightRadius/(rightRadius+leftRadius)
    
#Uses steering angle to calculate wheel-speed fraction from left-right
def calculateDiffRatio(wheelbase,steeringAngle,trackwidth):
    if(steeringAngle == 0):
        return 0.5
    rightRadius =wheelbase/(np.tan(steeringAngle))-trackwidth/2
    leftRadius = wheelbase/(np.tan(steeringAngle))+trackwidth/2

    return rightRadius/(leftRadius+rightRadius)

#Takes the right and left "resistances" and creates a torque bias ratio. 
#(Not working right now)
def perfectTorqueVector(leftI, rightI):
    return rightI/(leftI+rightI)



#Calculates output torque using left-right differential ratio.
#This is completely broken and will need to be rewritten
def simulatePerfectDifferential(inputTorque, ratio):
    rightTorque = inputTorque*ratio
    leftTorque = inputTorque*(1-ratio)
    return [leftTorque,rightTorque]


#Calculates wheelspeeds using left-right differential ratio. 
def simulateOpenDifferential(inputSpeed, ratio):
    rightSpeed = inputSpeed*ratio*2
    leftSpeed = inputSpeed*(1-ratio)*2
    return [leftSpeed,rightSpeed]

#Mirrors 3d vector across Y axis. (ISO coordinate system) 
#ISO coordinate system is +Z-up, +X-forward, +Y-Left
def mirrorY(vec):
    u = [0,0,0]
    u[0] = vec[0]
    u[1] = -vec[1]
    u[2] = vec[2]
    return u 
