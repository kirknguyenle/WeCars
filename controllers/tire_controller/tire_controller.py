from controller import Supervisor,Robot, GPS, Motor
import sys
import numpy
import math
TIME_STEP = 32

robot = Supervisor()  # create Robot instance
mc_node = robot.getFromDef('Contact1')
tire_node = robot.getFromDef('Wheel')

if mc_node == None:
    sys.exit(1)
    
coulomb_friction = mc_node.getField("coulombFriction")
sideslipConstant = mc_node.getField("forceDependentSlip")


data = robot.getDevice('gps')
data.enable(TIME_STEP)


slipAngle = 0.0



coulomb_friction.setMFFloat(0,0.2)

sideslipConstant.setMFFloat(0, 0.0)
sideslipConstant.setMFFloat(1, 0.0)




motor = robot.getDevice('AxleMotor')  # get the motor device


if motor == None:
    sys.exit(1)

motor.setPosition(float('inf')) 
motor.setVelocity(0.0)  # set the velocity to zero
i = 0

while robot.step(TIME_STEP) != -1:

    i +=1
    
    Max_Speed = 6.28
    U = data.getSpeedVector()
    wheelSpeed = Max_Speed * 0.5
    motor.setVelocity(wheelSpeed)  # roll forward
    tire_node.addForce([0, 0, 1], True)
    
    #print(velVec[1], velVec[2])
    
    if U[1] != 0.0:
    
        slipAngle = numpy.arctan(U[1]/U[0])
    

    print("Slip Angle: ", slipAngle*180/math.pi)
    
    
  # [CODE PLACEHOLDER 2]

  
    if i == 100: 
  
      val = 2.0
  
      coulomb_friction.setMFFloat(0,val)
      
      

  
    if i == 300: 
  
      side_val = 1
      
      sideslipConstant.setMFFloat(1, side_val)
      tire_node.addForce([0, 0, 20], True)

  
    if i == 400:
      new_val = 3.0
      coulomb_friction.setMFFloat(0,new_val)