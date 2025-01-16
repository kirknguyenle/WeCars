"""a_arm_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor
from numpy import *

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
motor = robot.getDevice('frontleft_suspension_motor')
stmotor = robot.getDevice('steer_left')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

simtime = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    simtime += timestep/1000.0
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    motor.setPosition(0.3*sin(6*simtime))
    #stmotor.setPosition(0.03*sin(3*simtime))
    

# Enter here exit cleanup code.
