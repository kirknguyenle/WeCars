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
#file = open(location, "w")

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

with open(r'C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\car_files\mr2.json') as f:
    data = json.load(f)

trackwidth = data["trackwidth"]
wheelbase = data["wheelbase"]

mc_nodes = [[robot.getFromDef(cO.createLocationString(1,1)+"_contact"),
            robot.getFromDef(cO.createLocationString(-1,1)+"_contact")],
            [robot.getFromDef(cO.createLocationString(1,-1)+"_contact"),
             robot.getFromDef(cO.createLocationString(-1,-1)+"_contact")]]



loads = [[robot.getDevice("flts"),
            robot.getDevice("frts")],
            [robot.getDevice("rlts"),
             robot.getDevice("rrts")]]
steeringRack = [robot.getDevice("steer_left"),
                robot.getDevice("steer_right")]



rearRods = [robot.getDevice("rear_static_left"),
            robot.getDevice("rear_static_right")]
if rearRods[0] == None:
     s.exit(1)

slipParams = [[mc_nodes[0][0].getField("forceDependentSlip"),
               mc_nodes[0][1].getField("forceDependentSlip")],
              [mc_nodes[1][0].getField("forceDependentSlip"),
               mc_nodes[1][1].getField("forceDependentSlip")]]

if robot.getDevice(cO.createLocationString(1,1)+"_drive_motor") == None:
    s.exit(1)

drives = [[robot.getDevice(cO.createLocationString(1,1)+"_drive_motor"),
            robot.getDevice(cO.createLocationString(-1,1)+"_drive_motor")],
            [robot.getDevice(cO.createLocationString(1,-1)+"_drive_motor"),
             robot.getDevice(cO.createLocationString(-1,-1)+"_drive_motor")]]

suspension = [[robot.getDevice(cO.createLocationString(1,1)+"_suspension_motor"),
            robot.getDevice(cO.createLocationString(-1,1)+"_suspension_motor")],
            [robot.getDevice(cO.createLocationString(1,-1)+"_suspension_motor"),
             robot.getDevice(cO.createLocationString(-1,-1)+"_suspension_motor")]]


accel = robot.getDevice("accel")
diff_cont = robot.getDevice("diffMan")
diff_cont.enable(timestep)
accel.enable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

for i in range(2):
    for j in range(2):
        drives[i][j].setPosition(float('inf'))
        drives[i][j].setVelocity(0.0)
        #drives[i][j].setAvailableTorque(20)

for i in range(2):
    for j in range(2):
        suspension[i][j].setPosition(0)
        suspension[i][j].setAvailableTorque(5000)


for i in range(2):
    steeringRack[i].setAvailableForce(300)
    rearRods[i].setAvailableForce(3000)
    rearRods[i].setPosition(0)

slip = [[0,0],[0,0]]

simtime = 0

driveVelocity = 2

speeds =[2, 2]

while robot.step(timestep) != -1:
    simtime += timestep/100

    print(round(simtime,2))
    drives[0][0].setAvailableTorque(0)
    drives[0][1].setAvailableTorque(0)

    
    accel_data = diff_cont.getValues()
    if(simtime > 10):
        steeringRack[0].setPosition(0.025)
        steeringRack[1].setPosition(0.025)
    
    #print(accel.getValues())
    #print('------------------------------------')
    #print(accel_data)
    #print('------------------------------------')
    #print(gps.getSpeedVector())
    #realspeed = gps.getSpeed()
    #print('------------------------------------')
    #print(realspeed)
    #print('------------------------------------')
    radius = np.pow(driveVelocity,2)/accel_data[1]
    diffRatio = cO.radiusDiffRatio(trackwidth, radius, np.sign(accel_data[1]))
   

    #print(np.round(speeds,2))
    
    driveRPM = driveVelocity/0.25
    if(round(simtime*100,2)%25==0):
        speeds = cO.simulateOpenDifferential(driveRPM, diffRatio)
    #print(speeds)
    if simtime > 2.5:
        drives[1][0].setTorque(300)
        drives[1][1].setTorque(300)

    if simtime > 10:
        driveVelocity = 4



    



