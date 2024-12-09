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

slipParams = [[mc_nodes[0,0].getField("forceDependentSlip"),
               mc_nodes[0,1].getField("forceDependentSlip")],
              [mc_nodes[1,0].getField("forceDependentSlip"),
               mc_nodes[1,1].getField("forceDependentSlip")]]

slip = [[0,0],[0,0]]

AppliedTorque = 0

while robot.step(timestep) != -1:
    steerSensor = steeringRack[0].getPositionSensor()
    
    ratio = cO.calculateDifferentialSpeeds