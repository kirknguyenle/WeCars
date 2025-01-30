from controller import Supervisor,Robot,GPS,Motor,Node
import numpy as np
import sys as s
import os.path
import carOrganizer as cO
import math 
from datetime import datetime as dt
import json 


run = dt.now()
runString= run.strftime("%d-%m-%Y-%H-%M-%S")

save_path = (r"C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\testData")
filename = (r"SLA_Car" + runString)
location = os.path.join(save_path, filename+".txt")
print(location)
file = open(location, "w")

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
trackwidth = 2
wheelbase = 6

mc_nodes = [[robot.getFromDef(cO.createLocationString(1,1)+"_contact"),
            robot.getFromDef(cO.createLocationString(-1,1)+"_contact")],
            [robot.getFromDef(cO.createLocationString(1,-1)+"_contact"),
             robot.getFromDef(cO.createLocationString(1,-1)+"_contact")]]
loads = [[robot.getDevice("frontLeft_Wheel_forceSensor"),
            robot.getDevice("frontRight_Wheel_forceSensor")],
            [robot.getDevice("rearLeft_Wheel_forceSensor"),
             robot.getDevice("rearRight_Wheel_forceSensor")]]
steeringRack = [robot.getDevice("steer_left"),
                robot.getDevice("steer_right")]
slipParams = [[mc_nodes[0][0].getField("forceDependentSlip"),
               mc_nodes[0][1].getField("forceDependentSlip")],
              [mc_nodes[1][0].getField("forceDependentSlip"),
               mc_nodes[1][1].getField("forceDependentSlip")]]
drives = [[robot.getFromDef(cO.createLocationString(1,1)+"_drive_motor"),
            robot.getFromDef(cO.createLocationString(-1,1)+"_drive_motor")],
            [robot.getFromDef(cO.createLocationString(1,-1)+"_drive_motor"),
             robot.getFromDef(cO.createLocationString(1,-1)+"_drive_motor")]]



slip = [[0,0],[0,0]]






