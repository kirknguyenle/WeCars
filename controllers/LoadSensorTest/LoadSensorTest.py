from controller import Robot, Motor,InertialUnit
from numpy import *
import sys

robot = Robot()
timestep = int(robot.getBasicTimeStep())

loads = robot.getDevice('touch sensor')

loads.enable(10)

loadMat = [0,0,0]

while robot.step(timestep) !=-1:
    l = loads.getValues()
    for i in range(3):
        loadMat[i] = l[i]

    #print(loadMat)