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

with open(r'C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\car_files\mr2\mr2.json') as f:
    data = json.load(f)

trackwidth = data["trackwidth"]
wheelbase = data["wheelbase"]
wheel_radius = data["tire_radius"]
tire_type = data["tire_type"]
cornering_stiffness = data["cornering_stiffness"]
magic_formula_constants = data["magic_formula_constants"]

rundrive = True
runsteer = True

steer_angles, rack_extension = np.loadtxt(r"C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\car_files\mr2\mr2Steer.txt", delimiter=',', unpack=True)

print(steer_angles)

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

brakes = [[robot.getDevice(cO.createLocationString(1,1)+"_brake"),
            robot.getDevice(cO.createLocationString(-1,1)+"_brake")],
            [robot.getDevice(cO.createLocationString(1,-1)+"_brake"),
             robot.getDevice(cO.createLocationString(-1,-1)+"_brake")]]

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
        loads[i][j].enable(50)

for i in range(2):
    for j in range(2):
        drives[i][j].setPosition(float('inf'))
        drives[i][j].setVelocity(0.0)
        #drives[i][j].setAvailableTorque(20)

for i in range(2):
    for j in range(2):
        suspension[i][j].setPosition(0)
        suspension[i][j].setAvailableTorque(0)


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
linWheelVel = [[0,0],[0,0]]

steerLeftRaw = [0,0,0]
steerRightRaw = [0,0,0]
steerLeftAdj = 0
steerRightAdj = 0

rotationMat = [[0,0],[0,0]]
rawSpeed = [0,0,0]
speed2D = [0,0]
speedRelative = [0,0]

cornerVelocities = [[[0,0],[0,0]],
                    [[0,0],[0,0]]]

frontLeftForces = [0,0,0]
frontRightForces = [0,0,0]
rearLeftForces = [0,0,0]
rearRightForces = [0,0,0]

worldYaw = 0
lastYaw = 0 
deltaYaw = 0
yawDot = 0

strRack = 0
strA = 0

mat = [[[0,0,0],[0,0,0],[0,0,0]],
       [[0,0,0],[0,0,0],[0,0,0]],
       [[0,0,0],[0,0,0],[0,0,0]]]

rawLoads = [[[0,0,0],[0,0,0]],
            [[0,0,0],[0,0,0]]]

verticalLoads = [[0,0],[0,0]]

testcounter = 0

while robot.step(timestep) != -1:
    simtime += timestep/1000
    drives[0][0].setAvailableTorque(0)
    drives[0][1].setAvailableTorque(0)


    #brakes[0][0].setDampingConstant(9000)
    #brakes[0][1].setDampingConstant(9000)

    if np.round(simtime*1000,0)%50 == 0:
        for j in range(2):
            for i in range(2):
                
                wheelEnc[j][i] = wheelSensors[j][i].getValue()
                wheelDelta[j][i] = wheelEnc[j][i]-wheelLast[j][i]
                wheelVel[j][i] = (wheelDelta[1][i]/(50))*1000
                wheelLast[j][i] = wheelEnc[j][i]

    for i  in range(2):
        for j in range(2):
            linWheelVel[i][j] = wheelVel[i][j]*wheel_radius
            

    for i in range(2):
        for j in range(2):
            local_var = loads[i][j].getValues()
            rawLoads[i][j] = [local_var[0], local_var[1], local_var[2]]

    
    for i in range(2):
        for j in range(2):
            mat[i][j] = th.rotationMatrix3D(wheelEnc[i][j])

    frontLeftForces = np.matmul(mat[0][0],rawLoads[0][0])
    frontRightForces = np.matmul(mat[0][1],rawLoads[0][1])
    rearLeftForces = np.matmul(mat[1][0],rawLoads[1][0])
    rearRightForces = np.matmul(mat[1][1],rawLoads[1][1])

    #print(simtime)
    #print('---------------------------')

    verticalLoads[0][0] = frontLeftForces[1]
    verticalLoads[0][1] = frontRightForces[1]
    verticalLoads[1][0] = rearLeftForces[1]
    verticalLoads[1][1] = rearRightForces[1]
    

    imuData = imu.getRollPitchYaw()
    if np.round(simtime*1000,0)%50 == 0:
        worldYaw = imuData[2]
        deltaYaw = worldYaw-lastYaw
        yawDot = (deltaYaw/50)*1000
        lastYaw = worldYaw



    rotationMat = [[np.cos(worldYaw),np.sin(worldYaw)],
                   [-np.sin(worldYaw),np.cos(worldYaw)]]

    rawSpeed = gps.getSpeedVector()

    for i in range(2):
        speed2D[i] = rawSpeed[i]

    speedRelative = np.matmul(rotationMat, speed2D)

    cornerVelocities[0][0] = th.velocityTransform2D(speedRelative, yawDot, [wheelbase/2, trackwidth/2])
    cornerVelocities[0][1] = th.velocityTransform2D(speedRelative, yawDot, [wheelbase/2, -trackwidth/2])
    cornerVelocities[1][0] = th.velocityTransform2D(speedRelative, yawDot, [-wheelbase/2, trackwidth/2])
    cornerVelocities[1][1] = th.velocityTransform2D(speedRelative, yawDot, [-wheelbase/2, -trackwidth/2])
    
    relativeVelAngle = np.arctan(speedRelative[1]/speedRelative[0])

    steerLeftRaw= steerSensors[0][0].getRollPitchYaw()
    steerRightRaw= steerSensors[0][1].getRollPitchYaw()
    
    steerLeftAdj = steerLeftRaw[2]-worldYaw
    steerRightAdj = steerRightRaw[2]-worldYaw

    slip[0][0] = np.arctan(cornerVelocities[0][0][1]/cornerVelocities[0][0][0])-steerLeftAdj  
    slip[0][1] = np.arctan(cornerVelocities[0][1][1]/cornerVelocities[0][1][0])-steerRightAdj
    for i in range(2):
        slip[1][i] = np.arctan(cornerVelocities[1][i][1]/cornerVelocities[0][i][0])
    

    for i in range(2):
        for j in range(2):
            match tire_type:
                case "linear Saturated":
                  SL = th.simpleLinearSaturatedLong(linWheelVel[i][j],speedRelative[0],cornering_stiffness, cornering_stiffness*0.9)
                  S = th.simpleLinearSaturated(speedRelative[0], cornering_stiffness, slip[i][j], cornering_stiffness*0.9)
                  mc_fields[i][j].setMFFloat(1, S)
                  mc_fields[i][j].setMFFloat(0, SL)
                case "magic formula":
                    S = th.simpleMFLat(magic_formula_constants[0], magic_formula_constants[1], magic_formula_constants[2], magic_formula_constants[3], slip[i][j], speedRelative[0])
                    SL = th.simpleMFLong(magic_formula_constants[0], magic_formula_constants[1], magic_formula_constants[2], magic_formula_constants[3], linWheelVel[i][j], speedRelative[0])
                    mc_fields[i][j].setMFFloat(1, S)
                    mc_fields[i][j].setMFFloat(0, SL)
                case "magic formula Fz":
                    S = th.mfzLat(magic_formula_constants[0], magic_formula_constants[1], magic_formula_constants[2], magic_formula_constants[3], slip[i][j], speedRelative[0], verticalLoads[i][j])
                    SL = th.mfzLong(magic_formula_constants[0], magic_formula_constants[1], magic_formula_constants[2], magic_formula_constants[3], linWheelVel[i][j], speedRelative[0], verticalLoads[i][j])
                    mc_fields[i][j].setMFFloat(1, S)
                    mc_fields[i][j].setMFFloat(0, SL)
                case _:
                  S = th.simpleLinearSaturated(speedRelative[0], cornering_stiffness, slip[i][j], cornering_stiffness*0.9)
                  mc_fields[i][j].setMFFloat(1, S)    


    #vh = np.round((wheelVel[1][0]+wheelVel[1][1])/2,2)

    accel_data = diff_cont.getValues()

    #radius = np.pow(vh,2)/np.round(accel_data[1],2)

    strRack = np.interp(0, steer_angles, rack_extension)

    print(strRack)
    print('---------------------------')


    steeringRack[0].setPosition(0)
    steeringRack[1].setPosition(strRack)

    #Steer System Identification: 

    if runsteer == False:
        if(simtime > 1.7):
            strA = 0.32
    
        if(simtime > 3.7):
            strA = 0.48
        if(simtime > 4.7):
            strA = -0.48

    #driveRPM = driveVelocity/0.25
    #if(round(simtime*100,2)%25==0):
        #speeds = cO.simulateOpenDifferential(driveRPM, diffRatio)
    if rundrive == True:
        if simtime > 1:
            drives[1][0].setTorque(250)
            drives[1][1].setTorque(250)
    
        if simtime > 4:
            drives[1][0].setTorque(400)
            drives[1][1].setTorque(400)


    

    



