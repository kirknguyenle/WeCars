from controller import Supervisor,Robot,GPS,Motor,Node
import numpy as np
import sys as s
import os.path

import math 
from datetime import datetime as dt
import json 

s.path.insert(0,r'C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\extra_scripts\SupportScripts')
import carOrganizer as cO
import tireHelper as th
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
tire_type = data["tire_type"]
cornering_stiffness = data["cornering_stiffness"]

mc_nodes = [[robot.getFromDef(cO.createLocationString(1,1)+"_contact"),
            robot.getFromDef(cO.createLocationString(-1,1)+"_contact")],
            [robot.getFromDef(cO.createLocationString(1,-1)+"_contact"),
             robot.getFromDef(cO.createLocationString(-1,-1)+"_contact")]]



mc_fields = [[0,0],[0,0]]

for i in range(2):
    for j in range(2):
        mc_fields[i][j] = mc_nodes[i][j].getField('forceDependentSlip')


loads = [[robot.getDevice("flts"),
            robot.getDevice("frts")],
            [robot.getDevice("rlts"),
             robot.getDevice("rrts")]]

steerSensors = [[robot.getDevice(cO.createLocationString(1,1)+"_inertial_name"),
                 robot.getDevice(cO.createLocationString(-1,1)+"_inertial_name")],
                [robot.getDevice(cO.createLocationString(1,-1)+"_inertial_name"),
                 robot.getDevice(cO.createLocationString(-1,-1)+"_inertial_name")]]

wheelSensors = [[robot.getDevice(cO.createLocationString(1,1)+"_encoder"),
                 robot.getDevice(cO.createLocationString(-1,1)+"_encoder")],
                [robot.getDevice(cO.createLocationString(1,-1)+"_encoder"),
                 robot.getDevice(cO.createLocationString(-1,-1)+"_encoder")]]

steeringRack = [robot.getDevice("steer_left"),
                robot.getDevice("steer_right")]



rearRods = [robot.getDevice("rear_static_left"),
            robot.getDevice("rear_static_right")]


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
diff_cont.enable(50)
accel.enable(10)
gps = robot.getDevice("gps")
gps.enable(timestep)
imu = robot.getDevice("imu")
imu.enable(timestep)

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
    steeringRack[i].setAvailableForce(6000)
    rearRods[i].setAvailableForce(40000)
    rearRods[i].setPosition(0)

for i in range(2):
    for j in range(2):
        wheelSensors[i][j].enable(5)
        steerSensors[i][j].enable(timestep)

slip = [[0,0],[0,0]]

simtime = 0

wheelEnc = [[0,0],[0,0]]
wheelLast = [[0,0],[0,0]]
wheelDelta = [[0,0],[0,0]]
wheelVel = [[0,0],[0,0]]

steerLeftRaw = [0,0,0]
steerRightRaw = [0,0,0]
steerLeftAdj = 0
steerRightAdj = 0

rotationMat = [[0,0],[0,0]]
rawSpeed = [0,0,0]
speed2D = [0,0]
speedRelative = [0,0]
relativeVelAngle = 0

while robot.step(timestep) != -1:
    simtime += timestep/1000
    drives[0][0].setAvailableTorque(0)
    drives[0][1].setAvailableTorque(0)
    if np.round(simtime*1000,0)%50 == 0: 
        for j in range(2):
            for i in range(2):
                wheelEnc[j][i] = wheelSensors[j][i].getValue()
                wheelDelta[j][i] = wheelEnc[j][i]-wheelLast[j][i]
                wheelVel[j][i] = wheelDelta[1][i]*1000
                wheelLast[j][i] = wheelEnc[j][i]

    imuData = imu.getRollPitchYaw()
    worldYaw = imuData[2]

    rotationMat = [[np.cos(worldYaw),np.sin(worldYaw)],
                   [-np.sin(worldYaw),np.cos(worldYaw)]]

    rawSpeed = gps.getSpeedVector()

    for i in range(2):
        speed2D[i] = rawSpeed[i]

    speedRelative = np.matmul(rotationMat, speed2D)
    relativeVelAngle = np.arctan(speedRelative[1]/speedRelative[0])


    steerLeftRaw= steerSensors[0][0].getRollPitchYaw()
    steerRightRaw= steerSensors[0][1].getRollPitchYaw()
    
    steerLeftAdj = steerLeftRaw[2]-worldYaw
    steerRightAdj = steerRightRaw[2]-worldYaw

    slip[0][0] = relativeVelAngle-steerLeftAdj  
    slip[0][1] = relativeVelAngle-steerRightAdj
    for i in range(2):
        slip[1][i] = relativeVelAngle
    
    print(round(slip[0][0],2))

    for i in range(2):
        for j in range(2):
            match tire_type:
                case "linear Saturated":
                  SL = th.simpleLinearSaturatedLong(wheelVel[i][j],speedRelative[0],cornering_stiffness, cornering_stiffness*0.9)
                  S = th.simpleLinearSaturated(speedRelative[0], cornering_stiffness, slip[i][j], cornering_stiffness*0.9)
                  mc_fields[i][j].setMFFloat(1, S)
                  mc_fields[i][j].setMFFloat(0,SL)
                case _:
                  S = th.simpleLinearSaturated(speedRelative[0], cornering_stiffness, slip[i][j], cornering_stiffness*0.9)
                  mc_fields[i][j].setMFFloat(1, S)    


    #vh = np.round((wheelVel[1][0]+wheelVel[1][1])/2,2)

    accel_data = diff_cont.getValues()

    #radius = np.pow(vh,2)/np.round(accel_data[1],2)




    if(simtime > 1):
        steeringRack[0].setPosition(0.025)
        steeringRack[1].setPosition(0.025)

    if(simtime > 2):
        steeringRack[0].setPosition(0.04)
        steeringRack[1].setPosition(0.04)
    if(simtime > 3):
        steeringRack[0].setPosition(-0.04)
        steeringRack[1].setPosition(-0.04)

    #driveRPM = driveVelocity/0.25
    #if(round(simtime*100,2)%25==0):
        #speeds = cO.simulateOpenDifferential(driveRPM, diffRatio)
    if simtime > 1.5:
        drives[1][0].setTorque(100)
        drives[1][1].setTorque(100)

    if simtime > 2.5:
        drives[1][0].setTorque(150)
        drives[1][1].setTorque(150)

        driveVelocity = 4



    



