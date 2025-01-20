"""a_arm_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor,InertialUnit
from numpy import *

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
motor = robot.getDevice('frontleft_suspension_motor')
stmotor = robot.getDevice('steer_left')
sensor = robot.getDevice('imu')


simtime = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    simtime += timestep/1000.0
    
    sensor.enable(timestep)
    data = sensor.getRollPitchYaw()
    print('toe: ', degrees(data[2]))
    
    if(simtime > 0.52):
        motor.setPosition(0.35)
    else:
        motor.setPosition(0.35*sin(3*simtime))
    #print('toe: ', data[2])

        
      
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.
    
    
    #stmotor.setPosition(0.03*sin(3*simtime))
    

# Enter here exit cleanup code.
