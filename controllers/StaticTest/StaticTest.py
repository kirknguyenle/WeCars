"""SuspGraph."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor,Robot, Motor,InertialUnit
import numpy as np
import math 
from datetime import datetime as dt
import sys as s
import os.path


run = dt.now()
runString= run.strftime("%d-%m-%Y-%H-%M-%S")

save_path = (r"C:\Users\minhk\OneDrive\Desktop\DriveLab\WeCars\testData")
filename = (r"Static" + runString)
location = os.path.join(save_path, filename+".txt")
print(location)
file = open(location, "w")
file.write("#Bump,Camber,Caster,Toe,SteerAngle,SteerRack \r\n")

breakline = "9,9,9,9,9,"+("9"*29)
# create the Robot instance.b
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())



# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
motor = robot.getDevice('frontleft_suspension_motor')
stmotor = robot.getDevice('steer_left')
tire_sensor = robot.getDevice('imu')
steer_sensor = robot.getDevice('frontleft_steerSensor')


itr = 0
reset = False
counter = 0
lastTime = 0
simtime = 0
test = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    simtime +=timestep/1000.0
    print(simtime)
    tire_sensor.enable(timestep)
    steer_sensor.enable(timestep)
    data = tire_sensor.getRollPitchYaw()

    #total of 40 datapoints per test
    jounceRange = np.arange(-0.5, 0.5, 0.025)
    steerRange = np.arange(-0.05, 0.05, 0.0025)

    file.write(
                str(simtime)+","+
                str(data[0])+","+
                str(data[1])+","+
                str(data[2])+","+
                str(steer_sensor.getValue())+","+
                "0.0"+"\r\n")
        

           
            
        

    



    
    

        
      

    

# Enter here exit cleanup code.
