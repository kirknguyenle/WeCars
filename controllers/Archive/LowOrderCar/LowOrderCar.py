from controller import Supervisor,Robot,GPS,Motor,Node
import numpy as np
import sys as s
import os.path
import carOrganizer as cO
import math 
from datetime import datetime as dt


run = dt.now()
runString= run.strftime("%d-%m-%Y-%H-%M-%S")

save_path = (r"C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\testData")
filename = (r"LowOrderCar" + runString)
location = os.path.join(save_path, filename+".txt")
print(location)
file = open(location, "w")

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

trackwidth = 2
wheelbase = 6

print (cO.createLocationString(1,-1)+"Drive")
print (cO.createLocationString(-1,-1)+"Drive")

mc_nodes = [[robot.getFromDef(cO.createLocationString(1,1)+"Contact"),
            robot.getFromDef(cO.createLocationString(-1,1)+"Contact")],
            [robot.getFromDef(cO.createLocationString(1,-1)+"Contact"),
             robot.getFromDef(cO.createLocationString(1,-1)+"Contact")]]

gps_nodes = [[robot.getDevice(cO.createLocationString(1,1)+"GPS"),
            robot.getDevice(cO.createLocationString(-1,1)+"GPS")],
            [robot.getDevice(cO.createLocationString(1,-1)+"GPS"),
             robot.getDevice(cO.createLocationString(1,-1)+"GPS")]]

loads = [[robot.getDevice(cO.createLocationString(1,1)+"Force"),
            robot.getDevice(cO.createLocationString(-1,1)+"Force")],
            [robot.getDevice(cO.createLocationString(1,-1)+"Force"),
             robot.getDevice(cO.createLocationString(1,-1)+"Force")]]

steeringRack = [robot.getDevice(cO.createLocationString(1,1)+"Steer"),
                robot.getDevice(cO.createLocationString(-1,1)+"Steer")]
steeringRackData = [[robot.getDevice(cO.createLocationString(1,1)+"SteerSensor"),
                    robot.getDevice(cO.createLocationString(-1,1)+"SteerSensor")]]

slipParams = [[mc_nodes[0][0].getField("forceDependentSlip"),
               mc_nodes[0][1].getField("forceDependentSlip")],
              [mc_nodes[1][0].getField("forceDependentSlip"),
               mc_nodes[1][1].getField("forceDependentSlip")]]

Drives = [[robot.getDevice(cO.createLocationString(1,-1)+"Drive"),robot.getDevice(cO.createLocationString(-1,-1)+"Drive")]]

slip = [[0,0],[0,0]]

steerAngle = np.radians(1.57/20)

AppliedTorque = 0
torques = [[0,0]]

while robot.step(timestep) != -1:
    steeringRackData[0][0].enable(timestep)
    #print (steeringRackData[0][0].getValue())
    ratio = cO.calculateDiffRatio(wheelbase,steerAngle, trackwidth)

    torques = cO.simulateDifferential(AppliedTorque, ratio)
    print(torques)   
    #Drives[0][0].setTorque(torques[0][0])
    #Drives[0][1].setTorque(torques[0][1])

    steeringRack[0].setPosition(steerAngle)
    steeringRack[1].setPosition(steerAngle)

    