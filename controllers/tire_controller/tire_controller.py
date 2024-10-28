from controller import Supervisor,Robot, Motor, GPS
import sys
import numpy
TIME_STEP = 32

robot = Supervisor()  # create Robot instance
mc_node = robot.getFromDef('Contact1')

if w_node == None:
    sys.exit(1)
    
coulomb_friction = mc_node.getField("coulombFriction")
sideslipConstant = mc_node.getField("forceDependentSlip")

data = robot.getGPS('gps')
data.enable(TIME_STEP)

slipAngle = 0.0

velVec = data.getSpeedVector()

coulomb_friction.setMFFloat(0,0.0)

sideslipConstant.setMFFloat(0, -1.0)




motor = robot.getDevice('motor')  # get the motor device
motor.setPosition(float('inf'))  
motor.setVelocity(0.0)  # set the velocity to zero
i = 0

while robot.step(TIME_STEP) != -1:
    Max_Speed = 6.28
    
    wheelSpeed = Max_Speed * 0.5
    motor.setVelocity(wheelSpeed)  # roll forward
    
    slipAngle = numpy.arctan(velVec[0]/velVec[2])
    
    print("Slip Angle: ", slipAngle)
    
    
  # [CODE PLACEHOLDER 2]

  
    if i == 100: 
  
      val = 2.0
  
      coulomb_friction.setMFFloat(0,val)
      

  
    if i == 300: 
  
      side_val = 1
      
      sideslipConstant.setMFFloat(0, side_val)
      

  
    if i == 400:
      new_val = 3.0
      coulomb_friction.setMFFloat(0,new_val)