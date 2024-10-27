from controller import Supervisor
import sys
import numpy
TIME_STEP = 32  

robot = Supervisor()  # create Supervisor instance

# [CODE PLACEHOLDER 1]
mc_node = robot.getFromDef('Contact1')
c_node = robot.getFromDef('Camera')
w_node = robot.getFromDef('Ground')


if w_node == None:
    sys.exit(1)



coulomb_friction = mc_node.getField("coulombFriction")
sideslipConstant = mc_node.getField("forceDependentSlip")
origin = w_node.getField('translation')

coulomb_friction.setMFFloat(0,0.0)

sideslipConstant.setMFFloat(0, -1.0)

i = 0
while robot.step(TIME_STEP) != -1:
  # [CODE PLACEHOLDER 2]

  i += 1
  if i < 900:
  
    origin.setSFVec3f([i/200, i/200, -0.431798])
  
  if i == 40: 
  
      val = 2.0
  
      coulomb_friction.setMFFloat(0,val)
      

  
  if i == 50: 
  
      side_val = 1
      
      sideslipConstant.setMFFloat(0, side_val)
      

  
  if i == 80:
      new_val = 3.0
      coulomb_friction.setMFFloat(0,new_val)
#while robot.step(TIME_STEP) != -1:
#while simtime<=12:
 #   simtime+=TIME_STEP/1000.0
    #if(i <5):
    
    #i += 1


