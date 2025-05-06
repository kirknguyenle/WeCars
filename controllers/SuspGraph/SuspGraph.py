"""SuspGraph."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor,Robot, Motor,InertialUnit
import numpy as np
import math 
from datetime import datetime as dt
import sys as s
import os.path
import matplotlib.pyplot as mpl
import matplotlib.animation as ani


run = dt.now()
runString= run.strftime("%d-%m-%Y-%H-%M-%S")

save_path = (r"C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\testData")
filename = (r"SuspGraph" + runString)
location = os.path.join(save_path, filename+".txt")
print(location)
#file = open(location, "w")
#file.write("#Bump,Camber,Caster,Toe,SteerAngle,SteerRack \r\n")

breakline = "9,9,9,9,9,"+("9"*29)
# create the Robot instance.b
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())



# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
drive = robot.getDevice('frontleft_drive_motor')
motor = robot.getDevice('frontleft_suspension_motor')
stmotor = robot.getDevice('steer_left')
tire_sensor = robot.getDevice('frontleft_inertial_name')
steer_sensor = robot.getDevice('frontleft_steerSensor')
tire_sensor.enable((timestep))
steer_sensor.enable((timestep))

counter = 0
simtime = 0


data = [0,0,0]



fig = mpl.figure()
ax = fig.add_subplot(1, 1, 1)

#mpl.ion()
#mpl.show()

t = 0
bmpt = []
cmb = []
pdif = 0
mdif = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller



while robot.step(timestep) != -1:
    lastTime = simtime 
    simtime +=timestep/100.0
    lastData = data
    data = tire_sensor.getRollPitchYaw()
    motor.setAvailableTorque(400)
    stmotor.setAvailableForce(0)
    
    #total of 400 datapoints per test
    jounceRange = np.arange(-0.35, 0.35, 0.0025)
    steerRange = np.arange(-0.05, 0.05, 0.00025)

    
    
  

    if simtime <5:
        drive.setVelocity(1)
        #motor.setPosition(-0.35)
        
    if 5<simtime < 383:
        drive.setVelocity(1)
        counter = math.ceil(simtime*100)-500
        #motor.setPosition(jounceRange[counter-2])
        #file.write(
        #            str(jounceRange[counter-1])+","+
        #            str(data[0])+","+
        #            str(data[1])+","+
        #            str(data[2])+","+
        #            str(steer_sensor.getValue())+","+
        #            "0.0"+"\r\n")
        

    
            
        

    




    

# Enter here exit cleanup code.
